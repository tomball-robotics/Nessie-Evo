package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElbowPivot extends SubsystemBase {

  private static SparkMax elbowPivot;
  private static SparkMaxConfig elbowPivotConfig;
  private static SparkLimitSwitch forwardLimitSwitch;
  private static SparkLimitSwitch reverseLimitSwitch;
  private static PIDController elbowPIDController;
  private static SparkAbsoluteEncoder absoluteEncoder;
  private static RelativeEncoder relativeEncoder;

  public ElbowPivot(){
    elbowPivot = new SparkMax(Constants.ElbowConstants.elbowPivotID, MotorType.kBrushless);
    forwardLimitSwitch = elbowPivot.getForwardLimitSwitch();
    reverseLimitSwitch = elbowPivot.getReverseLimitSwitch();

    elbowPivotConfig = new SparkMaxConfig();
    elbowPivotConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(Constants.ElbowConstants.smartCurrentLimit);
    elbowPivotConfig.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchEnabled(true)
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchEnabled(true);
    elbowPivotConfig.softLimit
      .forwardSoftLimit(Constants.ElbowConstants.forwardSoftLimit)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(Constants.ElbowConstants.reverseSoftLimit)
      .reverseSoftLimitEnabled(true);
    elbowPivot.configure(elbowPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elbowPIDController = new PIDController(
      Constants.ElbowConstants.p,
      Constants.ElbowConstants.i,
      Constants.ElbowConstants.d);

    elbowPIDController.setTolerance(Constants.ElbowConstants.tolerance);

    relativeEncoder = elbowPivot.getEncoder();
    relativeEncoder.setPosition(absoluteEncoder.getPosition() * 36);
  }

  public void setRotation (double angle){
    elbowPivot.set(elbowPIDController.calculate(relativeEncoder.getPosition()/36, angle));
  }

  public boolean elbowAtSetpoint() {
    return elbowPIDController.atSetpoint();
  }

  public void stopElbow() {
    elbowPivot.stopMotor();
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Elbow Absolute Position (Motor)", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Elbow Velocity (Motor)", absoluteEncoder.getVelocity());
    SmartDashboard.putNumber("Elbow Relative Position (Encoder)", relativeEncoder.getPosition());
    SmartDashboard.putBoolean("Elbow Forward Limit Reached", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Elbow Reverse Limit Reached", reverseLimitSwitch.isPressed());
  }

}
