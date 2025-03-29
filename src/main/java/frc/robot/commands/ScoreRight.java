package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.endeffector.IntakeAndHoldCoral;
import frc.robot.commands.endeffector.ShootCoral;
import frc.robot.commands.swerve.AlignToReefTagRelative;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;

public class ScoreRight extends SequentialCommandGroup {

  public ScoreRight(StateMachine stateMachine, Arm arm, Elevator elevator, Swerve swerve, EndEffector endEffector) {

    addCommands(
      new AlignToReefTagRelative("back", swerve).withTimeout(2),
      new InstantCommand(() -> stateMachine.requestState(stateMachine.desiredLevel)),
      new WaitCommand(2),
      new AlignToReefTagRelative("right", swerve),
      new IntakeAndHoldCoral(endEffector),
      new WaitCommand(1),
      new ShootCoral(endEffector),
      new WaitCommand(1),
      new AlignToReefTagRelative("back", swerve),
      new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW))
    );

  }

}
