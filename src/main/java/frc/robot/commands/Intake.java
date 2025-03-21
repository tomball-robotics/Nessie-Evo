package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollers;

public class Intake extends Command {

  private IntakeRollers intakeRollers;

  public Intake(IntakeRollers intakeRollers) {
    this.intakeRollers = intakeRollers;
  }

  @Override
  public void initialize() {
    intakeRollers.setBigRollerSpeed(.5);
    intakeRollers.setSmallRollerSpeed(.5);
    intakeRollers.setIndexerSpeed(.5);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    intakeRollers.stopAllMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
