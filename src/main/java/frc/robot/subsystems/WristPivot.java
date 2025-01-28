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

public class WristPivot extends SubsystemBase {

  private static SparkMax wristPivot;
  private static SparkMaxConfig wristPivotConfig;
  private static SparkLimitSwitch forwardLimitSwitch;
  private static SparkLimitSwitch reverseLimitSwitch;
  private static PIDController wristPIDController;
  private static SparkAbsoluteEncoder absoluteEncoder;
  private static RelativeEncoder relativeEncoder;

  public WristPivot(){
    wristPivot = new SparkMax(Constants.WristConstants.wristPivotID, MotorType.kBrushless);
    forwardLimitSwitch = wristPivot.getForwardLimitSwitch();
    reverseLimitSwitch = wristPivot.getReverseLimitSwitch();
    wristPivotConfig = new SparkMaxConfig();
    wristPivotConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(Constants.WristConstants.smartCurrentLimit);
    wristPivotConfig.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchEnabled(true)
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchEnabled(true);
    wristPivotConfig.softLimit
      .forwardSoftLimit(Constants.WristConstants.forwardSoftLimit)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(Constants.WristConstants.reverseSoftLimit)
      .reverseSoftLimitEnabled(true);
    wristPivot.configure(wristPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristPIDController = new PIDController(
      Constants.WristConstants.p,
      Constants.WristConstants.i,
      Constants.WristConstants.d);

    wristPIDController.setTolerance(Constants.WristConstants.tolerance);

    relativeEncoder = wristPivot.getEncoder();
    relativeEncoder.setPosition(absoluteEncoder.getPosition() * 36);
  }

  public void setRotation (double angle){
    wristPivot.set(wristPIDController.calculate(relativeEncoder.getPosition()/36, angle));
  }

  public boolean wristAtSetpoint() {
    return wristPIDController.atSetpoint();
  }

  public void stopWrist() {
    wristPivot.stopMotor();
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Wrist Absolute Position (Motor)", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Velocity (Motor)", absoluteEncoder.getVelocity());
    SmartDashboard.putNumber("Wrist Relative Position (Encoder)", relativeEncoder.getPosition());
    SmartDashboard.putBoolean("Wrist Forward Limit Reached", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Wrist Reverse Limit Reached", reverseLimitSwitch.isPressed());
  }

}
