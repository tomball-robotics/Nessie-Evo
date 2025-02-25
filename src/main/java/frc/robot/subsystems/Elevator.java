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
import frc.robot.RobotContainer;

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
    canandmag.setPosition(0);

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

  public void setDesiredPosition(double desiredPosition) {
    Elevator.desiredPosition = desiredPosition;
  }

  private void goToDesiredPosition() {
    double currentPosition = canandmag.getPosition();
    double output = controller.calculate(currentPosition, desiredPosition);
    setSpeed(output);
  }

  public void setSpeed(double desiredSpeed) {
    double currentPosition = canandmag.getPosition();
    double kG = Constants.ElevatorConstants.G; // gravity feedforward
    double kS = Constants.ElevatorConstants.S; // static friction compensation
    double forwardLimit = Constants.ElevatorConstants.FORWARD_LIMIT;
    double reverseLimit = Constants.ElevatorConstants.REVERSE_LIMIT;
    if(currentPosition >= forwardLimit && desiredSpeed > 0) {
      desiredSpeed = 0;
    }else if(currentPosition <= reverseLimit && desiredSpeed < 0) {
      desiredSpeed = 0;
    }
    motor.set(desiredSpeed + kS + kG);
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    if(!RobotContainer.manual) {
      goToDesiredPosition();
      SmartDashboard.putBoolean("Elevator at Setpoint", controller.atSetpoint());
      SmartDashboard.putNumber("Elevator Desired Position", desiredPosition);
    }
    SmartDashboard.putNumber("Elevator Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Elevator Desired Position", desiredPosition);
    SmartDashboard.putNumber("Elevator Motor Output", motor.get());
    SmartDashboard.putNumber("Elevator Position", canandmag.getPosition());
    SmartDashboard.putNumber("ELevator Supply Current", motor.getSupplyCurrent().getValueAsDouble());
  }
}
 