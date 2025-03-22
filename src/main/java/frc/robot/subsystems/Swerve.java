package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Elastic;
import frc.lib.LimelightHelpers;
import frc.lib.Elastic.Notification;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field;
    private SwerveModule[] swerveModules;
    private Pigeon2 gyro;
    private double speedMultiplier;
    public String desiredAlignment = "center";
    private StructPublisher<Pose2d> posePublisher;

    public Swerve() {
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID, "cani");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        speedMultiplier = 1.0;

        field = new Field2d();

        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
        );

        try {
            RobotConfig config;
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                this
            );
        } catch (Exception e) {
            Elastic.sendNotification(new Notification(Notification.NotificationLevel.ERROR, "Failed to configure AutoBuilder", "The AutoBuilder could not be configured. " + e.getMessage()));
        }

    }

    public void setDesiredAlignment(String desiredAlignment) {
        this.desiredAlignment = desiredAlignment;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX() * speedMultiplier,
                    translation.getY() * speedMultiplier,
                    rotation * speedMultiplier,
                    getHeading()
                )
                : new ChassisSpeeds(
                    translation.getX() * speedMultiplier,
                    translation.getY() * speedMultiplier,
                    rotation * speedMultiplier)
            );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void autoHeadingFix() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            double angle = getHeading().getDegrees() > 0 ? getHeading().getDegrees() - 180 : getHeading().getDegrees() + 180;
            setHeading(new Rotation2d(Units.degreesToRadians(angle)));
        }
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    public void setX() {
        swerveModules[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)), true);
        swerveModules[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)), true);
        swerveModules[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)), true);
        swerveModules[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)), true);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = getPose();


        updateOdometry();

        field.setRobotPose(currentPose);

        posePublisher.set(new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(currentPose.getRotation().getDegrees())));
        SmartDashboard.putBoolean("Swerve/Fast Mode", speedMultiplier == 1.0);
        SmartDashboard.putData("Swerve/Field", field);
    }

      @SuppressWarnings("removal")
    public void updateOdometry() {

        poseEstimator.update(
            gyro.getRotation2d(),
            getModulePositions()
        );

        boolean useMegaTag2 = false; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if(useMegaTag2 == false)
        {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

        if(mt1 == null) {
            DriverStation.reportWarning("Back Limelight Not Detected", false);
            return;
        }
        
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
            doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
            doRejectUpdate = true;
            }
        }
        if(mt1.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            poseEstimator.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
        }
        }
        else if (useMegaTag2 == true)
        {
        LimelightHelpers.SetRobotOrientation("limelight-back", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
        if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
        }
    }

}
