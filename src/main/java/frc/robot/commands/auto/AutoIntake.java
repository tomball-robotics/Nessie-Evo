package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.superstructure.EndEffector;

public class AutoIntake extends Command {

  private StateMachine stateMachine;
  private EndEffector endEffector;
  private Timer timer;

  public AutoIntake(StateMachine stateMachine, EndEffector endEffector) {
    this.stateMachine = stateMachine;
    this.endEffector = endEffector;
    timer = new Timer();

    addRequirements(stateMachine, endEffector);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return timer.get() > .3;
  }
}