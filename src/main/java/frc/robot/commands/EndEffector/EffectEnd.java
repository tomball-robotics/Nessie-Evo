package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffector;

public class EffectEnd extends Command {

  private EndEffector endEffector;
  private double speed;
  private boolean intake;

  public EffectEnd(EndEffector endEffector, boolean intake) {
    this.endEffector = endEffector;
    this.intake = intake;
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {
    if (intake) {
      if(RobotContainer.isAlgae) {
        speed = Constants.EndEffectorConstants.algaeIntakeSpeed;
      } else {
        speed = Constants.EndEffectorConstants.algaeOuttakeSpeed;
      }
    } else {
      if(!RobotContainer.isAlgae) {
        speed = Constants.EndEffectorConstants.coralOuttakeSpeed;
      } else {
        speed = Constants.EndEffectorConstants.coralIntakeSpeed;
      }
    }
  }

  @Override
  public void execute() {
    endEffector.runEndEffector(speed);
  }

  @Override
  public void end(boolean interrupted) {
    endEffector.stopEndEffector();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
