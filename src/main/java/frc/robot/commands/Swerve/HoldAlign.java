package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class HoldAlign extends Command {

  private Swerve swerve;
  private RobotContainer robotContainer;

  private String target;
  private double xSetpoint;
  private final double ySetpoint = 0;
  
  private final double rotationalP = 0.1;
  private final double translationalP = 0.1;
  private final double strafeP = 0.1;

  private final double translationTolerance = 0.1; // Adjust as needed
  private final double strafeTolerance = 0.1;      // Adjust as needed
  private final double rotationTolerance = 1.0;    // Adjust as needed (in degrees)

  public HoldAlign(Swerve swerve, RobotContainer robotContainer) {
    this.swerve = swerve;
    this.robotContainer = robotContainer;

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
    double[] targetPoseRobot = LimelightHelpers.getTargetPose_RobotSpace("limelight-front");
    if(aligned(targetPoseRobot)) {
      robotContainer.operator.setRumble(RumbleType.kBothRumble, 1);
      SmartDashboard.putBoolean("Swerve/Aligned", true);
    }else {
      SmartDashboard.putBoolean("Swerve/Aligned", false);
      robotContainer.operator.setRumble(RumbleType.kBothRumble, 0);
      if(LimelightHelpers.getTV("limelight-front")) {
        double xError = targetPoseRobot[0] - xSetpoint;
        double yError = targetPoseRobot[1] - ySetpoint;
        
        double yawToTag = targetPoseRobot[5];
        
        double translationOut = translationalP * xError;
        double strafeOut = strafeP * yError;
        double rotationOut = -rotationalP * yawToTag;
    
        swerve.drive(
            new Translation2d(translationOut, strafeOut).times(Constants.Swerve.maxSpeed), 
            rotationOut * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
      }
    }
  }

  private boolean aligned(double[] targetPoseRobot) {
    if(LimelightHelpers.getTV("limelight-front")) {
      double xError = Math.abs(targetPoseRobot[0] - xSetpoint);
      double yError = Math.abs(targetPoseRobot[1] - ySetpoint);
      double yawError = Math.abs(targetPoseRobot[5]);
      
      return (xError <= translationTolerance && yError <= strafeTolerance && yawError <= rotationTolerance);
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Swerve/Aligned", false);
    robotContainer.operator.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
