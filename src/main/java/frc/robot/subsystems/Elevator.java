package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMaster;
  private TalonFX elevatorSlave;
  private CANcoder absoluteEncoder;
  private PIDController elevatorPID;

  public Elevator() {
    elevatorMaster = new TalonFX(Constants.ElevatorConstants.elevatorMasterID);
    elevatorSlave = new TalonFX(Constants.ElevatorConstants.elevatorSlaveID);
    absoluteEncoder = new CANcoder(Constants.ElevatorConstants.canCoderID);

    elevatorMaster.setNeutralMode(NeutralModeValue.Brake);
    elevatorSlave.setNeutralMode(NeutralModeValue.Brake);

    elevatorPID = new PIDController(
      Constants.ElevatorConstants.p,
      Constants.ElevatorConstants.i,
      Constants.ElevatorConstants.d);
    elevatorPID.setTolerance(Constants.ElevatorConstants.tolerance);

    elevatorSlave.setControl(new Follower(elevatorMaster.getDeviceID(), false));
  }

  public void setPosition(double targetPosition){
      SmartDashboard.putNumber("Elevator Setpoint (Degrees)", targetPosition);
      double currentPosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
      elevatorMaster.set(elevatorPID.calculate(currentPosition, targetPosition));
  }

  public void stopElevator(){
    elevatorMaster.stopMotor();
  }

  public boolean atSetpoint(){
    return elevatorPID.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Velocity", absoluteEncoder.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
  }
}