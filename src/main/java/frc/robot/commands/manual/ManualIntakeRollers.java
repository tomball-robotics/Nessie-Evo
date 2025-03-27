package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollers;

public class ManualIntakeRollers extends Command {

  private IntakeRollers intakeRollers;
  private double voltageSup;
  private boolean index;

  public ManualIntakeRollers(IntakeRollers intakeRollers, double voltageSup, boolean index) {
    this.intakeRollers = intakeRollers;
    this.voltageSup = voltageSup;
    this.index = index;
  }

  @Override
  public void initialize() {
    intakeRollers.setBigRollerSpeed(voltageSup);
    if(index) {
      intakeRollers.setSmallRollerSpeed(-voltageSup);
      intakeRollers.setIndexerSpeed(-voltageSup);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intakeRollers.stopAllMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
