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

  private TalonFX elevatorMaster;
  private TalonFX elevatorSlave;
  private Canandmag canandmag;
  private PIDController elevatorPID;

  public Elevator() {
    elevatorMaster = new TalonFX(Constants.ElevatorConstants.elevatorMasterID);
    elevatorSlave = new TalonFX(Constants.ElevatorConstants.elevatorSlaveID);
    canandmag = new Canandmag(Constants.ElevatorConstants.CanandmagID);

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    elevatorConfig.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.elevatorCurrentLimit;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorMaster.getConfigurator().apply(elevatorConfig);

    elevatorPID = new PIDController(
      Constants.ElevatorConstants.p,
      Constants.ElevatorConstants.i,
      Constants.ElevatorConstants.d);
    elevatorPID.setTolerance(Constants.ElevatorConstants.tolerance);

    elevatorSlave.setControl(new Follower(elevatorMaster.getDeviceID(), false));
  }

  public void setPosition(double targetPosition) {
    double currentPosition = canandmag.getPosition();
    SmartDashboard.putNumber("Elevator Setpoint (Degrees)", targetPosition);
    
    if (currentPosition >= Constants.ElevatorConstants.forwardSoftLimit || 
        currentPosition <= Constants.ElevatorConstants.reverseSoftLimit) {
      stop();
    } else {
      elevatorMaster.set(elevatorPID.calculate(currentPosition, targetPosition));
    }
  }

  public void setSpeed(double desiredSpeed) {
    double currentPosition = canandmag.getPosition();
    
    if (currentPosition >= Constants.ElevatorConstants.forwardSoftLimit || 
        currentPosition <= Constants.ElevatorConstants.reverseSoftLimit) {
      stop();
    } else {
      elevatorMaster.set(desiredSpeed);
    }
  }

  public void stop() {
    elevatorMaster.stopMotor();
  }

  public boolean atSetpoint() {
    return elevatorPID.atSetpoint();
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Elevator Position (Radians)", canandmag.getPosition());
    SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());

    SmartDashboard.putBoolean("Elevator at Forward Limit", canandmag.getAbsPosition() >= Constants.ElevatorConstants.forwardSoftLimit);
    SmartDashboard.putBoolean("Elevator at Reverse Limit", canandmag.getAbsPosition() <= Constants.ElevatorConstants.reverseSoftLimit);
  }
}
