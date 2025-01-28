package frc.robot.commands.Positioning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElbowPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.WristPivot;
import frc.robot.RobotContainer;

public class SetPosition extends Command {

  private Elevator elevator;
  private ElbowPivot elbowPivot;
  private WristPivot wristPivot;
  private double[] positions;

  public SetPosition(Elevator elevator, ElbowPivot elbowPivot, WristPivot wristPivot, double[] positions) {
    this.elevator = elevator;
    this.elbowPivot = elbowPivot;
    this.wristPivot = wristPivot;
    this.positions = positions;

    addRequirements(elevator);
    addRequirements(elbowPivot);
    addRequirements(wristPivot);
  }

  @Override
  public void initialize() {
    if (positions[3] == 1) {
      RobotContainer.isAlgae = false;
    } else {
      RobotContainer.isAlgae = true;
    }
  }

  @Override
  public void execute() {
    elevator.setPosition(positions[0]);
    elbowPivot.setRotation(positions[1]);
    wristPivot.setRotation(positions[2]);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
    elbowPivot.stopElbow();
    wristPivot.stopWrist();
  }

  @Override
  public boolean isFinished() {
    return elbowPivot.elbowAtSetpoint() && wristPivot.wristAtSetpoint() && elevator.elevatorAtSetpoint();
  }
  
}
