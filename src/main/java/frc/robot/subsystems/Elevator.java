package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonFX motor;
  private TalonFX follower;
  private TalonFXConfiguration config;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private PIDController controller;
  
  public Elevator() {
    motor = new TalonFX(Constants.ID.ELEVATOR_MASTER_TALONFX_ID);
    follower = new TalonFX(Constants.ID.ELEVATOR_FOLLOWER_TALONFX_ID);
    canandmag = new Canandmag(Constants.ID.ELEVATOR_ENCODER_ID);
    canandmag.setPosition(0);

    controller = new PIDController(
      Constants.ElevatorConstants.P,
      Constants.ElevatorConstants.I,
      Constants.ElevatorConstants.D);
    controller.setTolerance(Constants.ElevatorConstants.TOLERANCE);

    config = new TalonFXConfiguration();

    canandmagSettings = new CanandmagSettings();

    canandmagSettings.setInvertDirection(false);

    config.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    follower.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(config);

    canandmag.setSettings(canandmagSettings);

    motor.setNeutralMode(NeutralModeValue.Brake);
    follower.setNeutralMode(NeutralModeValue.Brake);

    follower.setControl(new Follower(10, true));
  }

  public void goTowardsDesiredPosition() {
    double currentPosition = canandmag.getPosition();
    double output = controller.calculate(currentPosition);
    setSpeed(output);
  }

  public void setSetpoint(double desiredPosition) {
    controller.setSetpoint(desiredPosition);
  }

  public void setSpeed(double desiredSpeed) {
    double currentPosition = canandmag.getPosition();
    double forwardLimit = Constants.ElevatorConstants.FORWARD_LIMIT;
    double reverseLimit = Constants.ElevatorConstants.REVERSE_LIMIT;

    if(currentPosition >= forwardLimit && desiredSpeed > 0) {
      desiredSpeed = 0;
    }else if(currentPosition <= reverseLimit && desiredSpeed < 0) {
      desiredSpeed = 0;
    }

    motor.set(desiredSpeed + Constants.ElevatorConstants.G);
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    goTowardsDesiredPosition();
    SmartDashboard.putBoolean("Elevator/at Setpoint", controller.atSetpoint());
    SmartDashboard.putNumber("Elevator/Setpoint", controller.getSetpoint());
    SmartDashboard.putNumber("Elevator/Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Elevator/Motor Output", motor.get());
    SmartDashboard.putNumber("Elevator/Position", canandmag.getPosition());
    SmartDashboard.putNumber("Elevator/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Forward Limit", Constants.ElevatorConstants.FORWARD_LIMIT);
    SmartDashboard.putNumber("Elevator/Reverse Limit", Constants.ElevatorConstants.REVERSE_LIMIT);
  }

}
 