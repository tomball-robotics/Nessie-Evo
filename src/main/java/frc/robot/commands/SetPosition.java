package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.Position;

public class SetPosition extends Command {

  private Elevator elevator;
  private Elbow elbow;
  private Wrist wrist;
  private Position position;

  public SetPosition(Elevator elevator, Elbow elbow, Wrist wrist, Position position) {
    this.elevator = elevator;
    this.elbow = elbow;
    this.wrist = wrist;
    this.position = position;

    addRequirements(elevator);
    addRequirements(elbow);
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    elevator.setDesiredPosition(position.getElevatorPosition());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    elbow.setDesiredPosition(position.getElbowPosition());
    wrist.setDesiredPosition(position.getWristPosition());
  }

  @Override
  public boolean isFinished() {
    return elevator.atSetpoint();
  }

}