package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elbow extends SubsystemBase {

  private SparkMax elbowMotor;
  private SparkMaxConfig elbowMotorConfig;
  private PIDController pidController;
  private SparkAbsoluteEncoder absoluteEncoder;
  private RelativeEncoder relativeEncoder;
  private SparkLimitSwitch forwardLimitSwitch;
  private SparkLimitSwitch reverseLimitSwitch;

  public Elbow() {
    elbowMotor = new SparkMax(Constants.ElbowConstants.elbowMotorID, MotorType.kBrushless);

    // motor config
    elbowMotorConfig = new SparkMaxConfig();
    elbowMotorConfig.idleMode(IdleMode.kBrake);
    elbowMotorConfig.smartCurrentLimit(Constants.ElbowConstants.elbowCurrentLimit);

    // initialize and configure the limit switches
    forwardLimitSwitch = elbowMotor.getForwardLimitSwitch();
    reverseLimitSwitch = elbowMotor.getReverseLimitSwitch();
    elbowMotorConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    // configure the soft limits for the motor
    elbowMotorConfig.softLimit
        .forwardSoftLimit(Constants.ElbowConstants.forwardSoftLimit / 10)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(Constants.ElbowConstants.reverseSoftLimit / 10)
        .reverseSoftLimitEnabled(true);
    elbowMotor.configure(elbowMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // initialize the PID controller with the specified constants
    pidController = new PIDController(
      Constants.ElbowConstants.p, 
      Constants.ElbowConstants.i, 
      Constants.ElbowConstants.d);
    pidController.setTolerance(Constants.ElbowConstants.tolerance);

    // initialize the absolute and relative encoders
    absoluteEncoder = elbowMotor.getAbsoluteEncoder();
    relativeEncoder = elbowMotor.getEncoder();
    relativeEncoder.setPosition(absoluteEncoder.getPosition() * 36);
  }

  // set the rotation of the elbow to a desired position
  public void setRotation(double desiredPosition) {
    SmartDashboard.putNumber("Elbow Setpoint (Degrees)", desiredPosition);
    double currentPosition = relativeEncoder.getPosition() / 36;
    elbowMotor.set(pidController.calculate(currentPosition, desiredPosition));
  }

  // set the speed of the elbow motor directly
  public void setSpeed(double speed) {
    elbowMotor.set(speed);
  }

  // stop the elbow motor
  public void stop() {
    elbowMotor.stopMotor();
  }

  // get the current position of the elbow in degrees
  public double getPositionInDegrees() {
    return relativeEncoder.getPosition() * 10;
  }

  // check if the elbow is at the setpoint
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  // called periodically to update the SmartDashboard with the current status
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elbow Forward Limit Reached", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Elbow Reverse Limit Reached", reverseLimitSwitch.isPressed());
    SmartDashboard.putNumber("Elbow Position", getPositionInDegrees());
    SmartDashboard.putBoolean("Elbow at Setpoint", atSetpoint());
  }

}