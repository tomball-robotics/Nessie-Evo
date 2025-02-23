package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  private TalonFX motor;
  private Canandmag canandmag;
  private PIDController controller;
  private double desiredPosition = 0;

  public Wrist() {
    motor = new TalonFX(Constants.WristConstants.MOTOR_ID);
    canandmag = new Canandmag(Constants.WristConstants.ENCODER_ID);

    TalonFXConfiguration wristConfig = new TalonFXConfiguration();

    wristConfig.CurrentLimits.SupplyCurrentLimit = Constants.WristConstants.CURRENT_LIMIT;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(wristConfig);

    motor.setNeutralMode(NeutralModeValue.Brake);

    controller = new PIDController(
      Constants.WristConstants.P,
      Constants.WristConstants.I,
      Constants.WristConstants.D);
    controller.setTolerance(Constants.WristConstants.PID_TOLERANCE);
  }

  public void setDesiredPosition(double desiredPosition) {
    this.desiredPosition = desiredPosition;
  }

  public void setSpeed(double desiredSpeed) {
    motor.set(-desiredSpeed);
  }

  public void stop() {
    motor.stopMotor();
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Desired Position", desiredPosition);
    SmartDashboard.putNumber("Wrist Position", canandmag.getPosition());
    SmartDashboard.putBoolean("Wrist at Setpoint", atSetpoint());

    if(!Constants.ControlConstants.DEBUG) {
      double currentPosition = canandmag.getPosition();
      double out = controller.calculate(currentPosition, desiredPosition);
      motor.set(out);
    }
  }
}
