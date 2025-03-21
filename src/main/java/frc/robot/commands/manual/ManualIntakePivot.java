package frc.robot.commands.manual;

import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakePivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class ManualIntakePivot extends Command {   

    private IntakePivot IntakePivot;    
    private DoubleSupplier desiredSpeedSup;

    public ManualIntakePivot(IntakePivot IntakePivot, DoubleSupplier desiredSpeed) {
        this.IntakePivot = IntakePivot;
        this.desiredSpeedSup = desiredSpeed;
        addRequirements(IntakePivot);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double desiredSpeed = MathUtil.applyDeadband(desiredSpeedSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        IntakePivot.setSpeed(desiredSpeed);
    }
}