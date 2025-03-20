package frc.robot.commands.position;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivot;

public class SetIntakePivotPosition extends Command {

  private IntakePivot intakePivot;
  private double position;

  public SetIntakePivotPosition(IntakePivot intakePivot, double position) {
    this.position = position;
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakePivot.setSetpoint(position);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return intakePivot.atSetpoint();
  }
}