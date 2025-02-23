package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class ManualEndEffector extends Command {

  private EndEffector endEffector;
  private DoubleSupplier speed;

  public ManualEndEffector(EndEffector endEffector, DoubleSupplier speed) {
    this.endEffector = endEffector;
    this.speed = speed;
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    endEffector.setSpeed(MathUtil.applyDeadband(speed.getAsDouble(),
      Constants.ControlConstants.STICK_DEADBAND));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
