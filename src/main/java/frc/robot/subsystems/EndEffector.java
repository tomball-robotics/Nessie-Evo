package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  private TalonFX endEffector;
  private TalonFXConfiguration endEffectorConfig;

  public EndEffector() {
    endEffector = new TalonFX(Constants.EndEffectorConstants.MOTOR_ID);
    endEffectorConfig = new TalonFXConfiguration();
    endEffectorConfig.CurrentLimits.SupplyCurrentLimit = Constants.EndEffectorConstants.CURRENT_LIMIT;
    endEffectorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    endEffector.setNeutralMode(NeutralModeValue.Brake);
    endEffector.getConfigurator().apply(endEffectorConfig);
  }

  public void runEndEffector(double speed) {
    endEffector.set(speed);
  }

  public void stopEndEffector() {
    endEffector.stopMotor();
  }

  @Override
  public void periodic() {}

}