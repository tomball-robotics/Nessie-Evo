package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoCoralOuttake;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.commands.position.SetElevatorPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ScoreL3 extends SequentialCommandGroup {

  private double elevatorHeight = 0; // TODO tune
  private double armExtendedPosition = 0; // TODO tune

  public ScoreL3(Arm arm, Elevator elevator, EndEffector endEffector) {

    addCommands(
      new SetElevatorPosition(elevator, elevatorHeight),
      new SetArmPosition(arm, armExtendedPosition),
      new AutoCoralOuttake(endEffector),
      new SetArmPosition(arm, 0),
      new SetElevatorPosition(elevator, 0)
    );

  }

}
