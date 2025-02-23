package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class ManualWrist extends Command {

  private Wrist wrist;
  private DoubleSupplier speed;

  public ManualWrist(Wrist wrist, DoubleSupplier speed) {
    this.wrist = wrist;
    this.speed = speed;
    addRequirements(wrist);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wrist.setSpeed(MathUtil.applyDeadband(speed.getAsDouble(),
      Constants.ControlConstants.STICK_DEADBAND));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
