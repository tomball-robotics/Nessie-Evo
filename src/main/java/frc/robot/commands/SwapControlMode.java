package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SwapControlMode extends Command {

  private boolean previous;

  public SwapControlMode() {}

  @Override
  public void initialize() {
    previous = RobotContainer.Manual;
    RobotContainer.Manual = !previous;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return previous != RobotContainer.Manual;
  }
}
