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

public class Elbow extends SubsystemBase {

  private static SparkMax motor;
  private static SparkMaxConfig config;
  private static PIDController controller;
  private static RelativeEncoder relativeEncoder;
  private static AbsoluteEncoder absoluteEncoder;
  private static SparkLimitSwitch forwardLimitSwitch;
  private static SparkLimitSwitch reverseLimitSwitch;
  private static double desiredPosition = 0;

  public Elbow() {
    motor = new SparkMax(Constants.ElbowConstants.MOTOR_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    forwardLimitSwitch = motor.getForwardLimitSwitch();
    reverseLimitSwitch = motor.getReverseLimitSwitch();
    absoluteEncoder = motor.getAbsoluteEncoder();
    relativeEncoder = motor.getEncoder();

    relativeEncoder.setPosition(absoluteEncoder.getPosition());

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.ElbowConstants.CURRENT_LIMIT);
    
    config.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchEnabled(true)
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchEnabled(true);

    config.softLimit
      .forwardSoftLimit(Constants.ElbowConstants.FORWARD_LIMIT)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(Constants.ElbowConstants.REVERSE_LIMIT)
      .reverseSoftLimitEnabled(true);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller = new PIDController(
      Constants.ElbowConstants.P,
      Constants.ElbowConstants.I,
      Constants.ElbowConstants.D);
    controller.setTolerance(Constants.ElbowConstants.PID_TOLERANCE);
  }

  public void setRotation(double desiredPosition) {
    Elbow.desiredPosition = desiredPosition;
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
    if(!Constants.ControlConstants.TEST_MODE) {
      double currentPosition = relativeEncoder.getPosition();
      double out = controller.calculate(currentPosition, desiredPosition);
      motor.set(out);
    }

    SmartDashboard.putNumber("Elbow Position", relativeEncoder.getPosition());
    SmartDashboard.putBoolean("Elbow at Setpoint", atSetpoint());
    SmartDashboard.putNumber("Elbow Setpoint", controller.getSetpoint());
    SmartDashboard.putNumber("Elbow Forward Limit", Constants.ElbowConstants.FORWARD_LIMIT);
    SmartDashboard.putNumber("Elbow Reverse Limit", Constants.ElbowConstants.REVERSE_LIMIT);
    SmartDashboard.putBoolean("Elbow at Forward Limit", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Elbow at Reverse Limit", reverseLimitSwitch.isPressed());
  }

}