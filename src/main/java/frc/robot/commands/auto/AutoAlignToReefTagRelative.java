package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoAlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private Swerve swerve;
  private double tagID = -1;

  public AutoAlignToReefTagRelative(boolean isRightScore, Swerve swerve) {
    xController = new PIDController(0, 0, 0);  // Vertical movement
    yController = new PIDController(0, 0, 0);  // Horitontal movement
    rotController = new PIDController(.1, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(0);
    rotController.setTolerance(1);

    xController.setSetpoint(-0.34);
    xController.setTolerance(0.02);

    yController.setSetpoint(isRightScore ? 0.16 : -0.16);
    yController.setTolerance(0.02);

    tagID = LimelightHelpers.getFiducialID("limelight-front");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-front") && LimelightHelpers.getFiducialID("limelight-front") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-front");
      SmartDashboard.putNumber("Align/X Position", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("Align/X Speed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      swerve.drive(
        new Translation2d(xSpeed, ySpeed).times(Constants.SwerveConstants.maxSpeed), 
        rotValue * Constants.SwerveConstants.maxAngularVelocity, 
        false, 
        true
      );

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
        SmartDashboard.putBoolean("Align/Aligned", false);
      }else {
        SmartDashboard.putBoolean("Align/Aligned", true);
      }

    } else {
      swerve.drive(
        new Translation2d().times(Constants.SwerveConstants.maxSpeed), 
        0 * Constants.SwerveConstants.maxAngularVelocity, 
        false, 
        true
      );
    }

    SmartDashboard.putNumber("Align/Pose Valid Timer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(
      new Translation2d().times(Constants.SwerveConstants.maxSpeed), 
      0 * Constants.SwerveConstants.maxAngularVelocity, 
      false, 
      true
    );
  }

  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(1) ||
        stopTimer.hasElapsed(.3);
  }
}