package frc.robot.commands.motions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoCoralOuttake;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.commands.position.SetElevatorPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ScoreL4 extends SequentialCommandGroup {

  private double elevatorHeight = 0; // TODO tune
  private double armExtendedPosition = 0; // TODO tune
  private double armClearancePosition = 0.437;

  public ScoreL4(Arm arm, Elevator elevator, EndEffector endEffector) {

    addCommands(
      new SetArmPosition(arm, armClearancePosition),
      new SetElevatorPosition(elevator, elevatorHeight),
      new SetArmPosition(arm, armExtendedPosition),
      new WaitCommand(.1),
      new AutoCoralOuttake(endEffector),
      new SetArmPosition(arm, armClearancePosition),
      new SetElevatorPosition(elevator, 0),
      new SetArmPosition(arm, 0)
    );

  }

}
