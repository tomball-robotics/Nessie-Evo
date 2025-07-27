package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Swerve.AlignToReefTagRelative;
import frc.robot.commands.auto.AutoCoralIntake;
import frc.robot.commands.auto.AutoShootCoral;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;

public class ScoreRight extends SequentialCommandGroup {

  public ScoreRight(StateMachine stateMachine, Arm arm, Elevator elevator, Swerve swerve, EndEffector endEffector) {

    addCommands(
      new AlignToReefTagRelative("right back", swerve).withTimeout(1),
      new WaitCommand(.5),
      new InstantCommand(() -> stateMachine.requestState(stateMachine.desiredLevel)),
      new WaitCommand(.5),
      new AlignToReefTagRelative("right forward", swerve).withTimeout(1),
      new WaitCommand(1),
      new AutoCoralIntake(endEffector),
      new WaitCommand(.5),
      new AutoShootCoral(endEffector),
      new WaitCommand(.5),
      new AlignToReefTagRelative("right back", swerve).withTimeout(1),
      new WaitCommand(.5),
      new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW))
    );

  }

}
