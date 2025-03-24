package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollers;

public class ManualIntakeRollers extends Command {

  private IntakeRollers intakeRollers;
  private double speedSup;
  private boolean index;

  public ManualIntakeRollers(IntakeRollers intakeRollers, double speedSup, boolean index) {
    this.intakeRollers = intakeRollers;
    this.speedSup = speedSup;
    this.index = index;
  }

  @Override
  public void initialize() {
    intakeRollers.setBigRollerSpeed(speedSup);
    if(index) {
      intakeRollers.setSmallRollerSpeed(speedSup);
      intakeRollers.setIndexerSpeed(speedSup);
    }
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
