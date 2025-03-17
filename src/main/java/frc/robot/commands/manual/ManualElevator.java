package frc.robot.commands.manual;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class ManualElevator extends Command {   

    private Elevator Elevator;    
    private DoubleSupplier desiredSpeedSup;

    public ManualElevator(Elevator Elevator, DoubleSupplier desiredSpeed) {
        this.Elevator = Elevator;
        this.desiredSpeedSup = desiredSpeed;
        addRequirements(Elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double desiredSpeed = MathUtil.applyDeadband(desiredSpeedSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        Elevator.setSpeed(desiredSpeed);
    }
}