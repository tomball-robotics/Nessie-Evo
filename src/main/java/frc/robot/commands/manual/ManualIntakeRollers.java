package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollers;

public class ManualIntakeRollers extends Command {

  private IntakeRollers intakeRollers;
  private double speedSup;

  public ManualIntakeRollers(IntakeRollers intakeRollers, double speedSup) {
    this.intakeRollers = intakeRollers;
    this.speedSup = speedSup;
  }

  @Override
  public void initialize() {
    intakeRollers.setBigRollerSpeed(speedSup);
    intakeRollers.setSmallRollerSpeed(speedSup);
    intakeRollers.setIndexerSpeed(speedSup);
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
