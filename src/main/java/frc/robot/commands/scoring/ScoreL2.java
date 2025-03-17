package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoCoralOuttake;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.commands.position.SetElevatorPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ScoreL2 extends SequentialCommandGroup {

  private double elevatorHeight = 1; // TODO tune
  private double armExtendedPosition = .7; // TODO tune
  private double armClearancePosition = .192;

  public ScoreL2(Arm arm, Elevator elevator, EndEffector endEffector) {

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
