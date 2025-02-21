package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  private static SparkMax motor;
  private static SparkMaxConfig config;
  private static PIDController controller;
  private static RelativeEncoder relativeEncoder;
  private static AbsoluteEncoder absoluteEncoder;
  private static SparkLimitSwitch forwardLimitSwitch;
  private static SparkLimitSwitch reverseLimitSwitch;

  public Wrist() {
    motor = new SparkMax(Constants.WristConstants.MOTOR_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    forwardLimitSwitch = motor.getForwardLimitSwitch();
    reverseLimitSwitch = motor.getReverseLimitSwitch();
    absoluteEncoder = motor.getAbsoluteEncoder();
    relativeEncoder = motor.getEncoder();

    relativeEncoder.setPosition(absoluteEncoder.getPosition());

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.WristConstants.CURRENT_LIMIT);
    
    config.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchEnabled(true)
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchEnabled(true);

    config.softLimit
      .forwardSoftLimit(Constants.WristConstants.FORWARD_LIMIT)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(Constants.WristConstants.REVERSE_LIMIT)
      .reverseSoftLimitEnabled(true);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller = new PIDController(
      Constants.WristConstants.P,
      Constants.WristConstants.I,
      Constants.WristConstants.D);
    controller.setTolerance(Constants.WristConstants.PID_TOLERANCE);
  }

  public void setRotation(double desiredPosition) {
    double currentPosition = relativeEncoder.getPosition();
    double out = controller.calculate(currentPosition, desiredPosition);
    motor.set(out);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", relativeEncoder.getPosition());
    SmartDashboard.putBoolean("Wrist at Setpoint", atSetpoint());
    SmartDashboard.putNumber("Wrist Setpoint", controller.getSetpoint());
    SmartDashboard.putNumber("Wrist Forward Limit", Constants.WristConstants.FORWARD_LIMIT);
    SmartDashboard.putNumber("Wrist Reverse Limit", Constants.WristConstants.REVERSE_LIMIT);
    SmartDashboard.putBoolean("Wrist at Forward Limit", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Wrist at Reverse Limit", reverseLimitSwitch.isPressed());
  }

}