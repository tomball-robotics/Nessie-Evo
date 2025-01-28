package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  TalonFX endEffector;
  TalonFXConfiguration endEffectorConfig;

  public EndEffector() {
    endEffector = new TalonFX(Constants.EndEffectorConstants.endEffectorID);
    endEffectorConfig = new TalonFXConfiguration();
    endEffectorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
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
  public void periodic() {

  }

}