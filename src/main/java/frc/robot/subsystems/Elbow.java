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

public class Elbow extends SubsystemBase {

  private static SparkMax elbowMotor;
  private static SparkMaxConfig elbowMotorConfig;
  private static PIDController pidController;
  private static SparkAbsoluteEncoder absoluteEncoder;
  private static RelativeEncoder relativeEncoder;

  public Elbow() {
    elbowMotor = new SparkMax(Constants.ElbowConstants.elbowMotorID, MotorType.kBrushless);
    elbowMotorConfig = new SparkMaxConfig();
    elbowMotorConfig.idleMode(IdleMode.kBrake);
    elbowMotor.configure(elbowMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController = new PIDController(
      Constants.ElbowConstants.p, 
      Constants.ElbowConstants.i, 
      Constants.ElbowConstants.d);
    pidController.setTolerance(Constants.ElbowConstants.tolerance);

    absoluteEncoder = elbowMotor.getAbsoluteEncoder();

    relativeEncoder = elbowMotor.getEncoder();
    relativeEncoder.setPosition(absoluteEncoder.getPosition() * 36);
  }

  public void setRotation(double desiredPosition) {

    /* send to smartdashboard */
    SmartDashboard.putNumber("Elbow Setpoint (Degrees)", desiredPosition);

    /* get absolute position from CANcoder in degrees */
    double currentPosition = relativeEncoder.getPosition() * 10;

    /* set motor speed to PID output */
    elbowMotor.set(pidController.calculate(currentPosition, desiredPosition));

  }

  public void setSpeed(double speed) {
    elbowMotor.set(speed);
  }

  public void stop() {
    elbowMotor.stopMotor();
  }

  public double getPositionInDegrees() {
    return relativeEncoder.getPosition() * 10;
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elbow Position", getPositionInDegrees());
    SmartDashboard.putBoolean("Elbow at Setpoint", atSetpoint());
  }

}
 