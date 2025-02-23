package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;

  public Climber() {
    motor = new TalonFX(Constants.ClimberConstants.MOTOR_ID);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = Constants.ClimberConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }
  
  public void runClimber(double desiredSpeed){
    motor.set(desiredSpeed);
  }

  public void stop(){
    motor.stopMotor();
  }   

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Velocity", motor.getVelocity().getValueAsDouble());
  }

}