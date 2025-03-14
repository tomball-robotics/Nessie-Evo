package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class CoralOuttake extends Command {

  private EndEffector endEffector;
  private double speed;

  public CoralOuttake(EndEffector endEffector) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {
    if(SmartDashboard.getString("Current Position", "").equals("L1")) {
      speed = .1;
    }else {
      speed = Constants.EndEffectorConstants.CORAL_OUTTAKE_SPEED;
    }
  }

  @Override
  public void execute() {
    endEffector.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    endEffector.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
