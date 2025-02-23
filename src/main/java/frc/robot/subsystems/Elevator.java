package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private static TalonFX motor;
  private static TalonFX follower;
  private static TalonFXConfiguration config;
  private static Canandmag canandmag;
  private static PIDController controller;
  private static double desiredPosition = 0;

  public Elevator() {
    motor = new TalonFX(Constants.ElevatorConstants.MASTER_ID);
    follower = new TalonFX(Constants.ElevatorConstants.FOLLOWER_ID);
    canandmag = new Canandmag(Constants.ElevatorConstants.ENCODER_ID);

    controller = new PIDController(
      Constants.ElevatorConstants.P,
      Constants.ElevatorConstants.I,
      Constants.ElevatorConstants.D);
    controller.setTolerance(Constants.ElevatorConstants.TOLERANCE);

    config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    motor.setNeutralMode(NeutralModeValue.Brake);
    follower.setNeutralMode(NeutralModeValue.Brake);

    follower.setControl(new Follower(10, false));
  }

  private void goToDesiredPosition() {
    double currentPosition = canandmag.getPosition();
    double output = controller.calculate(currentPosition, desiredPosition);
    motor.set(output + Constants.ElevatorConstants.G);
  }

  public void setDesiredPosition(double desiredPosition) {
    Elevator.desiredPosition = desiredPosition;
  }

  public void setSpeed(double desiredSpeed) {
    double governor;
    if(canandmag.getPosition() < .3) {
      governor = 10;
    }else {
      governor = 4;
    }
    motor.set((desiredSpeed/governor) + Constants.ElevatorConstants.G);
    SmartDashboard.putNumber("Elevator Desired Speed", desiredSpeed);
  }

  private boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    if(!Constants.ControlConstants.MANUAL_OPERATION) {
      goToDesiredPosition();
    }
    SmartDashboard.putNumber("Elevator Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Elevator Desired Position", desiredPosition);
    SmartDashboard.putNumber("Elevator Motor Output", motor.get());
    SmartDashboard.putNumber("Elevator Position", canandmag.getPosition());
    SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
  }
}
