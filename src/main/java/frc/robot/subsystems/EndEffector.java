package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;

  public EndEffector() {
    motor = new TalonFX(Constants.EndEffectorConstants.MOTOR_ID);
    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = Constants.EndEffectorConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(config);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Endeffector Velocity", motor.getVelocity().getValueAsDouble());
  }
  
}