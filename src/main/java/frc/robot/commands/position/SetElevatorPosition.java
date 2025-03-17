package frc.robot.commands.position;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends Command {

  private Elevator elevator;
  private double position;

  public SetElevatorPosition(Elevator elevator, double position) {
    this.position = position;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.goTowardsDesiredPosition(position);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return elevator.atSetpoint();
  }
}
