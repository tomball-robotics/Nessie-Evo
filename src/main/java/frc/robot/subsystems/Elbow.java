package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elbow extends SubsystemBase {

  private TalonFX motor;
  private Canandmag canandmag;
  private PIDController controller;
  private double desiredPosition = 0;

  public Elbow() {
    motor = new TalonFX(Constants.ElbowConstants.MOTOR_ID);
    canandmag = new Canandmag(Constants.ElbowConstants.ENCODER_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = Constants.ElbowConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = Constants.ElbowConstants.P;
    config.Slot0.kI = Constants.ElbowConstants.I;
    config.Slot0.kD = Constants.ElbowConstants.D;
    config.Slot0.kG = Constants.ElbowConstants.G;

    motor.getConfigurator().apply(config);

    motor.setNeutralMode(NeutralModeValue.Brake);

    controller = new PIDController(
      Constants.ElbowConstants.P,
      Constants.ElbowConstants.I,
      Constants.ElbowConstants.D);
    controller.setTolerance(Constants.ElbowConstants.PID_TOLERANCE);
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
    SmartDashboard.putNumber("Elbow Desired Position", desiredPosition);
    SmartDashboard.putNumber("Elbow Position", canandmag.getPosition());
    SmartDashboard.putBoolean("Elbow at Setpoint", atSetpoint());

    if(!Constants.ControlConstants.DEBUG) {
      double currentPosition = canandmag.getPosition();
      double out = controller.calculate(currentPosition, desiredPosition);
      motor.set(out);
    }
  }
}
