package frc.robot.commands.Positioning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.lib.Elastic;
import frc.lib.Elastic.Notification;
import frc.lib.Elastic.Notification.NotificationLevel;
import frc.robot.RobotContainer;

public class SetPosition extends Command {

  private Elevator elevator;
  private Elbow elbow;
  private Wrist wrist;
  private double[] positions;

  public SetPosition(Elevator elevator, Elbow elbow, Wrist wrist, double[] positions) {
    this.elevator = elevator;
    this.elbow = elbow;
    this.wrist = wrist;
    this.positions = positions;

    addRequirements(elevator);
    addRequirements(elbow);
    addRequirements(wrist);
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
    elbow.setRotation(positions[1]);
    wrist.setRotation(positions[2]);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    elbow.stop();
    wrist.stop();
    Elastic.sendNotification(new Notification(
      NotificationLevel.INFO,
      "Superstructure In Position",
      "The elbow, wrist, and elevator have each reached their desired positions of " + positions[0] + ", " + positions[1] + ", and " + positions[2] + " respectivly."));
  }

  @Override
  public boolean isFinished() {
    return elbow.atSetpoint() && wrist.atSetpoint() && elevator.atSetpoint();
  }
  
}
