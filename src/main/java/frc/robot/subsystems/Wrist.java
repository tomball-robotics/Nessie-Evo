package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {

  private static TalonFX motor;
  private static TalonFXConfiguration config;
  private static Canandmag canandmag;
  private static PIDController controller;
  private static double desiredPosition = 0;

  public Wrist() {
    motor = new TalonFX(Constants.WristConstants.MOTOR_ID);
    canandmag = new Canandmag(Constants.WristConstants.ENCODER_ID);
    canandmag.setPosition(0);

    controller = new PIDController(
      Constants.WristConstants.P,
      Constants.WristConstants.I,
      Constants.WristConstants.D);
    controller.setTolerance(Constants.WristConstants.TOLERANCE);

    config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = Constants.WristConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  private void goToDesiredPosition() {
    double currentPosition = canandmag.getPosition();
    double output = controller.calculate(currentPosition, desiredPosition);
    motor.set(output);
  }

  public void setDesiredPosition(double desiredPosition) {
    Wrist.desiredPosition = desiredPosition;
  }

  public void setSpeed(double desiredSpeed) {
    motor.set(desiredSpeed);
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    if(!RobotContainer.Manual) {
      goToDesiredPosition();
      SmartDashboard.putNumber("Wrist Desired Position", desiredPosition);
      SmartDashboard.putBoolean("Wrist at Setpoint", controller.atSetpoint());
    }
    SmartDashboard.putNumber("Wrist Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Wrist Motor Output", motor.get());
    SmartDashboard.putNumber("Wrist Position", canandmag.getPosition());
    SmartDashboard.putNumber("Wrist Supply Current", motor.getSupplyCurrent().getValueAsDouble());
  }
}
