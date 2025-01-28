package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonFX climberMotor;
  private TalonFXConfiguration climberConfig;

  public Climber() {
    climberMotor = new TalonFX(Constants.ClimberConstants.climberMotorID);
    climberMotor.setNeutralMode(NeutralModeValue.Brake);

    climberConfig = new TalonFXConfiguration();
    climberConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor.getConfigurator().apply(climberConfig);
  }
  
  public void runClimber(double speed){
    climberMotor.set(speed);
  }

  public void stopClimber(){
    climberMotor.stopMotor();
  }   

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Current Limit", climberConfig.CurrentLimits.SupplyCurrentLimit);
    SmartDashboard.putNumber("Climber Velocity", climberMotor.getVelocity().getValueAsDouble());
  }

}