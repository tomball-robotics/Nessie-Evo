package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.Position;

public class SetDesiredPosition extends Command {

  private Elevator elevator;
  private Elbow elbow;
  private Wrist wrist;
  private Position position;

  public SetDesiredPosition(Elevator elevator, Elbow elbow, Wrist wrist, Position position) {
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
    SmartDashboard.putString("Current Position", position.getName());
    elevator.setDesiredPosition(position.getElevatorPosition());
    elbow.setDesiredPosition(position.getElbowPosition());
    wrist.setDesiredPosition(position.getWristPosition());
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

}