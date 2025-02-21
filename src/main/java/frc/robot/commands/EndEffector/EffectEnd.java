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
        speed = Constants.EndEffectorConstants.ALGAE_INTAKE_SPEED;
      } else {
        speed = Constants.EndEffectorConstants.ALGAE_OUTTAKE_SPEED;
      }
    } else {
      if(!RobotContainer.isAlgae) {
        speed = Constants.EndEffectorConstants.CORAL_OUTTAKE_SPEED;
      } else {
        speed = Constants.EndEffectorConstants.CORAL_INTAKE_SPEED;
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
