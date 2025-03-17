package frc.robot.commands.motions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoCoralOuttake;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EndEffector;

public class ScoreL1 extends SequentialCommandGroup {

  private double armExtendedPosition = .25; // TODO tune

  public ScoreL1(Arm arm, EndEffector endEffector) {

    addCommands(
      new SetArmPosition(arm, armExtendedPosition), 
      new AutoCoralOuttake(endEffector), 
      new SetArmPosition(arm, 0)
    );

  }

}
