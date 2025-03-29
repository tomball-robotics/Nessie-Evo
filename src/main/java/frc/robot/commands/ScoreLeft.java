package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.AlignToReefTagRelative;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.superstructure.EndEffector;

public class ScoreLeft extends SequentialCommandGroup {

  public ScoreLeft(StateMachine stateMachine, Swerve swerve, EndEffector endEffector) {

    addCommands(
      new AlignToReefTagRelative("back", swerve),
      new InstantCommand(() -> stateMachine.requestState(stateMachine.getDesiredLevel())),
      new AlignToReefTagRelative("left", swerve),
      new WaitCommand(.5),
      new AutoShootCoral(endEffector),
      new WaitCommand(.5),
      new AlignToReefTagRelative("back", swerve),
      new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW))
    );

  }

}
