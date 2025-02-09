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

public class Wrist extends SubsystemBase {

  private SparkMax wristMotor;
  private SparkMaxConfig wristMotorConfig;
  private PIDController pidController;
  private SparkAbsoluteEncoder absoluteEncoder;
  private RelativeEncoder relativeEncoder;
  private SparkLimitSwitch forwardLimitSwitch;
  private SparkLimitSwitch reverseLimitSwitch;

  public Wrist() {
    wristMotor = new SparkMax(Constants.WristConstants.wristMotorID, MotorType.kBrushless);

    // motor config
    wristMotorConfig = new SparkMaxConfig();
    wristMotorConfig.idleMode(IdleMode.kBrake);
    wristMotorConfig.smartCurrentLimit(Constants.WristConstants.wristCurrentLimit);

    // initialize and configure the limit switches
    forwardLimitSwitch = wristMotor.getForwardLimitSwitch();
    reverseLimitSwitch = wristMotor.getReverseLimitSwitch();
    wristMotorConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    // configure the soft limits for the motor
    wristMotorConfig.softLimit
        .forwardSoftLimit(Constants.WristConstants.forwardSoftLimit / 10)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(Constants.WristConstants.reverseSoftLimit / 10)
        .reverseSoftLimitEnabled(true);
    wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // initialize the PID controller with the specified constants
    pidController = new PIDController(
      Constants.WristConstants.p, 
      Constants.WristConstants.i, 
      Constants.WristConstants.d);
    pidController.setTolerance(Constants.WristConstants.tolerance);

    // initialize the absolute and relative encoders
    absoluteEncoder = wristMotor.getAbsoluteEncoder();
    relativeEncoder = wristMotor.getEncoder();
    relativeEncoder.setPosition(absoluteEncoder.getPosition() * 36);
  }

  // set the rotation of the wrist to a desired position
  public void setRotation(double desiredPosition) {
    SmartDashboard.putNumber("Wrist Setpoint (Degrees)", desiredPosition);
    double currentPosition = relativeEncoder.getPosition() / 36;
    wristMotor.set(pidController.calculate(currentPosition, desiredPosition));
  }

  // set the speed of the wrist motor directly
  public void setSpeed(double speed) {
    wristMotor.set(speed);
  }

  // stop the wrist motor
  public void stop() {
    wristMotor.stopMotor();
  }

  // get the current position of the wrist in degrees
  public double getPositionInDegrees() {
    return relativeEncoder.getPosition() * 10;
  }

  // check if the wrist is at the setpoint
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  // called periodically to update the SmartDashboard with the current status
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Wrist Forward Limit Reached", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Wrist Reverse Limit Reached", reverseLimitSwitch.isPressed());
    SmartDashboard.putNumber("Wrist Position", getPositionInDegrees());
    SmartDashboard.putBoolean("Wrist at Setpoint", atSetpoint());
  }

}