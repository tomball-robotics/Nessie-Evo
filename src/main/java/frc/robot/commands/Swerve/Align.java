package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class Align extends Command {

  private Swerve swerve;
  private String target;
  private double xSetpoint;
  private final double ySetpoint = 0;
  
  private final double rotationalP = 0.1;
  private final double translationalP = 0.1;
  private final double strafeP = 0.1;

  public Align(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    target = swerve.desiredAlignment;

    if(target.equals("right")) {
      xSetpoint = 0;
    } else if(target.equals("left")) {
      xSetpoint = 0;
    } else if(target.equals("center")) {
      xSetpoint = 0;
    }

  }

  @Override
  public void execute() {
    if(LimelightHelpers.getTV("limelight-front")) {
      double[] targetPoseRobot = LimelightHelpers.getTargetPose_RobotSpace("limelight-front");
      
      // Extract x and y positions from targetPoseRobot
      double xError = targetPoseRobot[0] - xSetpoint;
      double yError = targetPoseRobot[1] - ySetpoint;
      
      // Yaw error for rotation
      double yawToTag = targetPoseRobot[5];
      
      // Calculate outputs
      double translationOut = translationalP * xError;
      double strafeOut = strafeP * yError;
      double rotationOut = -rotationalP * yawToTag;
  
      // Apply the calculated values to drive the swerve
      swerve.drive(
          new Translation2d(translationOut, strafeOut).times(Constants.Swerve.maxSpeed), 
          rotationOut * Constants.Swerve.maxAngularVelocity, 
          false, 
          true
      );
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
