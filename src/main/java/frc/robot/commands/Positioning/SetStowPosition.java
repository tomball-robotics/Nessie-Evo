package frc.robot.commands.Positioning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;
import frc.robot.Position;

public class SetStowPosition extends Command {

  private Elevator elevator;
  private Elbow elbow;
  private Wrist wrist;
  private Position position;

  public SetStowPosition(Elevator elevator, Elbow elbow, Wrist wrist) {
    this.elevator = elevator;
    this.elbow = elbow;
    this.wrist = wrist;
    this.position = Constants.PositionConstants.STOW;

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
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

}