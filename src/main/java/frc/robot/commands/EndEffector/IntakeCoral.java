package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class IntakeCoral extends Command {

  private EndEffector endEffector;

  public IntakeCoral(EndEffector endEffector) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    endEffector.setSpeed(Constants.EndEffectorConstants.CORAL_INTAKE_SPEED);
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
