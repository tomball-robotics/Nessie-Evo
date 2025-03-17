package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Elastic;
import frc.lib.Elastic.Notification;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers;

public class Swerve extends SubsystemBase {
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModule[] swerveModules;
    private Pigeon2 gyro;
    private double speedMultiplier;
    public String desiredAlignment = "center";
    private StructPublisher<Pose2d> posePublisher;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "cani");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        speedMultiplier = 1.0;

        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
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

        LimelightHelpers.setLEDMode_ForceOff("limelight-back");
        LimelightHelpers.setPipelineIndex("limelight-back", 0); // Ensure AprilTag pipeline is selected
        LimelightHelpers.SetIMUMode("limelight-back", 2); // Use internal IMU for MegaTag2
    }

    public void setDesiredAlignment(String desiredAlignment) {
        this.desiredAlignment = desiredAlignment;
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(fieldRelative
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

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
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
        poseEstimator.update(getGyroYaw(), getModulePositions());

        updateVisionMeasurement();

        posePublisher.set(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(getPose().getRotation().getDegrees())));
        SmartDashboard.putBoolean("Swerve/FastMode", speedMultiplier == 1.0);
        SmartDashboard.putString("Swerve/DesiredAlignment", desiredAlignment);
        SmartDashboard.putBoolean("Swerve/LeftDesiredAlign", desiredAlignment.equals("left") || desiredAlignment.equals("center"));
        SmartDashboard.putBoolean("Swerve/RightDesiredAlign", desiredAlignment.equals("right") || desiredAlignment.equals("center"));
    }

    @SuppressWarnings("removal")
    private void updateVisionMeasurement() {
        // Set the robot's orientation for MegaTag2
        LimelightHelpers.SetRobotOrientation("limelight-back", getGyroYaw().getDegrees(), 0, 0, 0, 0, 0);

        // Get the MegaTag2 pose estimate
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

        // Check if mt2 is null before accessing its fields
        if (mt2 == null) {
            // Optionally log a message for debugging
            DriverStation.reportWarning("Vision measurement returned null", false);
            return;
        }

        boolean doRejectUpdate = false;

        // Reject the update if the robot is rotating too quickly
        if (Math.abs(gyro.getRate()) > 720) { // 720 degrees per second
            doRejectUpdate = true;
        }

        // Reject the update if no tags are visible
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }

        // If the update is not rejected, add it to the pose estimator
        if (!doRejectUpdate) {
            poseEstimator.setVisionMeasurementStdDevs(edu.wpi.first.math.VecBuilder.fill(0.7, 0.7, 9999999));
            poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds
            );
        }
    }

}
