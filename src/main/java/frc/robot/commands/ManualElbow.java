package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elbow;

public class ManualElbow extends Command {

  private Elbow elbow;
  private DoubleSupplier speed;

  public ManualElbow(Elbow elbow, DoubleSupplier speed) {
    this.elbow = elbow;
    this.speed = speed;
    addRequirements(elbow);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elbow.setSpeed(MathUtil.applyDeadband(speed.getAsDouble(),
      Constants.ControlConstants.STICK_DEADBAND));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
