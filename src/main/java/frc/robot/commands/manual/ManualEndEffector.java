package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.EndEffector;

public class ManualEndEffector extends Command {

  private EndEffector endEffector;
  private double speedSup;

  public ManualEndEffector(EndEffector endEffector, double speedSup) {
    this.endEffector = endEffector;
    this.speedSup = speedSup;
  }

  @Override
  public void initialize() {
    endEffector.setSpeed(speedSup);
  }

  @Override
  public void execute() {
    
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
