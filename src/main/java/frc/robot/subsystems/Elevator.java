package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMaster;
  private TalonFX elevatorSlave;
  private CANcoder elevatorEncoder;
  private TalonFXConfiguration elevatorConfig;
  private PIDController elevatorPID;

  public Elevator() {
    elevatorMaster = new TalonFX(Constants.ElevatorConstants.elevatorMasterID);
    elevatorSlave = new TalonFX(Constants.ElevatorConstants.elevatorSlaveID);

    elevatorPID = new PIDController(
      Constants.ElevatorConstants.p, 
      Constants.ElevatorConstants.i, 
      Constants.ElevatorConstants.d);

    elevatorPID.setTolerance(Constants.ElevatorConstants.tolerance);

    elevatorMaster.setNeutralMode(NeutralModeValue.Brake);
    elevatorSlave.setNeutralMode(NeutralModeValue.Brake);

    elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.supplyCurrentLimit;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMaster.getConfigurator().apply(elevatorConfig);
    elevatorSlave.getConfigurator().apply(elevatorConfig);

    elevatorSlave.setControl(new Follower(elevatorMaster.getDeviceID(), false));
  }

  public void setPosition(double position){
    //elevatorMaster.set(elevatorPID.calculate(relativeEncoder.getPosition()/36, angle));
  }

  public void stopElevator(){
    elevatorMaster.stopMotor();
  }

  public boolean elevatorAtSetpoint(){
    return elevatorPID.atSetpoint();
  }

  @Override
  public void periodic() {

  }

}