package frc.robot.commands.Positioning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elbow;

public class ElbowUp extends Command {

  private Elbow elbow;

  public ElbowUp(Elbow elbow) {
    this.elbow = elbow;
    addRequirements(elbow);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elbow.setSpeed(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    elbow.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
