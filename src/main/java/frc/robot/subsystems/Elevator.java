package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMaster;
  private TalonFX elevatorSlave;
  private Canandmag canandmag;
  private PIDController controller;
  private ElevatorFeedforward feedforward;
  private double desiredPosition = 0;

  public Elevator() {
    elevatorMaster = new TalonFX(Constants.ElevatorConstants.MASTER_ID);
    elevatorSlave = new TalonFX(Constants.ElevatorConstants.FOLLOWER_ID);
    canandmag = new Canandmag(Constants.ElevatorConstants.ENCODER_ID);

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    elevatorConfig.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorMaster.getConfigurator().apply(elevatorConfig);
    elevatorSlave.getConfigurator().apply(elevatorConfig);

    elevatorMaster.setNeutralMode(NeutralModeValue.Brake);
    elevatorSlave.setNeutralMode(NeutralModeValue.Brake);

    controller = new PIDController(
      Constants.ElevatorConstants.P,
      Constants.ElevatorConstants.I,
      Constants.ElevatorConstants.D);
    controller.setTolerance(Constants.ElevatorConstants.PID_TOLERANCE);

    feedforward = new ElevatorFeedforward(
      Constants.ElevatorConstants.S,
      Constants.ElevatorConstants.V,
      Constants.ElevatorConstants.A,
      Constants.ElevatorConstants.G);

    elevatorSlave.setControl(new Follower(elevatorMaster.getDeviceID(), false));
  }

  public void setPosition(double targetPosition) {
    double feedforwardOutput = feedforward.calculate(canandmag.getVelocity());
    double pidOutput = controller.calculate(canandmag.getPosition(), targetPosition);
    double totalOutput = pidOutput + feedforwardOutput;
    elevatorMaster.set(totalOutput);
  }

  public void setDesiredPosition(double desiredPosition) {
    this.desiredPosition = desiredPosition;
  }

  public void setSpeed(double desiredSpeed) {
    double feedforwardOutput = feedforward.calculate(canandmag.getVelocity());
    elevatorMaster.set(-desiredSpeed + feedforwardOutput);
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    if(!Constants.ControlConstants.DEBUG) {
      setPosition(desiredPosition);
    }
    SmartDashboard.putNumber("Elevator Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Elevator Desired Position", desiredPosition);
    SmartDashboard.putNumber("Elevator Motor Output", elevatorMaster.get());
    SmartDashboard.putNumber("Elevator Position", canandmag.getPosition());
    SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
  }
}
