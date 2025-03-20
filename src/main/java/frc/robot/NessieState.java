package frc.robot;

public class NessieState {

    private double armPosition;
    private double elevatorPosition;
    private double intakePosition;
    private String name;

    public NessieState(String name, double armDesiredPosition, double elevatorDesiredPosition, double intakeDesiredPosition) {
        this.name = name;
        this.armPosition = armDesiredPosition;
        this.elevatorPosition = elevatorDesiredPosition;
        this.intakePosition = intakeDesiredPosition;
    }

    public double getArmPosition() {
        return armPosition;
    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }

    public double getIntakePosition() {
        return intakePosition;
    }

    public String getName() {
        return name;
    }

    public boolean isElevated() {
        return elevatorPosition > 0.1;
    }

    public boolean isExtended() {
        return armPosition > 0.1;
    }

    public boolean isIntakeExtended() {
        return intakePosition > 0.1;
    }
    
}
