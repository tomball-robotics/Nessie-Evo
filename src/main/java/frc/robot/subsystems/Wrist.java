package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  private static SparkMax wristMotor;
  private static SparkMaxConfig wristMotorConfig;
  private static PIDController pidController;
  private static SparkAbsoluteEncoder absoluteEncoder;
  private static RelativeEncoder relativeEncoder;

  public Wrist() {
    wristMotor = new SparkMax(Constants.WristConstants.wristMotorID, MotorType.kBrushless);
    wristMotorConfig = new SparkMaxConfig();
    wristMotorConfig.idleMode(IdleMode.kBrake);
    wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController = new PIDController(
      Constants.WristConstants.p, 
      Constants.WristConstants.i, 
      Constants.WristConstants.d);
    pidController.setTolerance(Constants.WristConstants.tolerance);

    absoluteEncoder = wristMotor.getAbsoluteEncoder();

    relativeEncoder = wristMotor.getEncoder();
    relativeEncoder.setPosition(absoluteEncoder.getPosition() * 36);
  }

  public void setRotation(double desiredPosition) {

    /* send to smartdashboard */
    SmartDashboard.putNumber("Wrist Setpoint (Degrees)", desiredPosition);

    /* get absolute position from CANcoder in degrees */
    double currentPosition = relativeEncoder.getPosition() * 10;

    /* set motor speed to PID output */
    wristMotor.set(pidController.calculate(currentPosition, desiredPosition));

  }

  public void setSpeed(double speed) {
    wristMotor.set(speed);
  }

  public void stop() {
    wristMotor.stopMotor();
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position (Degrees)", relativeEncoder.getPosition() * 10);
    SmartDashboard.putBoolean("Wrist at Setpoint", pidController.atSetpoint());
  }

}
 