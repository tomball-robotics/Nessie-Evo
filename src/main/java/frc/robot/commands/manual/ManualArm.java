package frc.robot.commands.manual;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class ManualArm extends Command {   

    private Arm arm;    
    private DoubleSupplier desiredSpeedSup;

    public ManualArm(Arm arm, DoubleSupplier desiredSpeed) {
        this.arm = arm;
        this.desiredSpeedSup = desiredSpeed;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double desiredSpeed = MathUtil.applyDeadband(desiredSpeedSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        arm.setSpeed(desiredSpeed);
    }
}