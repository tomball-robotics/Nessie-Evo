package frc.robot;

public class Position {

    private double elevator;
    private double elbow;
    private double wrist;

    private String name;
    private boolean coral;

    public Position(double elevator, double elbow, double wrist, boolean coral, String name) {
        this.elevator = elevator;
        this.elbow = elbow;
        this.wrist = wrist;
        this.name = name;
        this.coral = coral;
    }

    public String getName() {
        return name;
    }

    public double getElevatorPosition() {
        return elevator;
    }

    public double getElbowPosition() {
        return elbow;
    }
    
    public double getWristPosition() {
        return wrist;
    }

    public boolean isCoral() {
        return coral;
    }
}