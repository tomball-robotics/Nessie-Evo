package frc.robot.commands.swerve;

import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    

    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier lockSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier leftAlignSup;
    private BooleanSupplier rightAlignSup;
    private BooleanSupplier centerAlignSup;

    private PIDController xController, yController, rotController;

    public TeleopSwerve(
        Swerve swerve, 
        DoubleSupplier translationSup, 
        DoubleSupplier strafeSup, 
        DoubleSupplier rotationSup, 
        BooleanSupplier lockSup, 
        BooleanSupplier robotCentricSup,
        BooleanSupplier leftAlignSup,
        BooleanSupplier rightAlignSup,
        BooleanSupplier centerAlignSup
    ) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.lockSup = lockSup;
        this.robotCentricSup = robotCentricSup;
        this.leftAlignSup = leftAlignSup;
        this.rightAlignSup = rightAlignSup;
        this.centerAlignSup = centerAlignSup;

        xController = new PIDController(.6, 0, 0);  // Vertical movement
        yController = new PIDController(1, 0, 0);  // Horitontal movement
        rotController = new PIDController(.01, 0, 0);  // Rotation
        
    }

    @Override
    public void initialize() {
        rotController.setSetpoint(0);
        rotController.setTolerance(1);

        xController.setSetpoint(-0.31961580231078157);
        xController.setTolerance(0.02);

        yController.setTolerance(0.02);
    }

    @Override
    public void execute() {

        double x = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        double y = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        double rot = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        boolean robotCentric = robotCentricSup.getAsBoolean();

        if(leftAlignSup.getAsBoolean() || rightAlignSup.getAsBoolean() || centerAlignSup.getAsBoolean()) {
            double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-front");
            SmartDashboard.putNumber("Align/X Position", postions[2]);
            SmartDashboard.putNumber("Align/Y Position", postions[0]);
            SmartDashboard.putNumber("Align/Rot Position", postions[4]);

            if(leftAlignSup.getAsBoolean()) {
                yController.setSetpoint(-0.14479634513709055);
            }else if(rightAlignSup.getAsBoolean()) {
                yController.setSetpoint(0.14479634513709055);
            }else {
                yController.setSetpoint(0);
            }

            double xSpeed = xController.calculate(postions[2]);
            double ySpeed = -yController.calculate(postions[0]);
            double rotSpeed = -rotController.calculate(postions[4]);

            x += xSpeed;
            y += ySpeed;
            rot += rotSpeed;
            robotCentric = true;
        }

        if(!lockSup.getAsBoolean()) {
            swerve.drive(
                new Translation2d(x, y).times(Constants.SwerveConstants.maxSpeed), 
                rot * Constants.SwerveConstants.maxAngularVelocity, 
                !robotCentric, 
                true
            );
        }else {
            swerve.setX();
        }
    }
} 