package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.EndEffector;

public class ManualEndEffector extends Command {

  private EndEffector endEffector;
  private double voltageSup;

  public ManualEndEffector(EndEffector endEffector, double voltageSup) {
    this.endEffector = endEffector;
    this.voltageSup = voltageSup;
  }

  @Override
  public void initialize() {
    endEffector.setVoltage(voltageSup);
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
