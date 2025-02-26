package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elbow extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private PIDController controller;
  private double desiredPosition = 0;

  public Elbow() {
    motor = new TalonFX(Constants.ElbowConstants.MOTOR_ID);
    config = new TalonFXConfiguration();
    canandmag = new Canandmag(Constants.ElbowConstants.ENCODER_ID);
    canandmagSettings = new CanandmagSettings();
    canandmagSettings.setInvertDirection(true);

    controller = new PIDController(
      Constants.ElbowConstants.P,
      Constants.ElbowConstants.I,
      Constants.ElbowConstants.D);
    controller.setTolerance(Constants.ElbowConstants.TOLERANCE);

    config.CurrentLimits.SupplyCurrentLimit = Constants.ElbowConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    canandmag.setSettings(canandmagSettings);
    canandmag.setPosition(0);
  }

  public void setDesiredPosition(double desiredPosition) {
    this.desiredPosition = desiredPosition;
  }

  private void goToDesiredPosition() {
    double currentPosition = canandmag.getPosition();
    double output = controller.calculate(currentPosition, desiredPosition);
    setSpeed(output);
  }

  public void setSpeed(double desiredSpeed) {
    double currentPosition = canandmag.getPosition();
    double feedForward = -0.02179303089 * Math.sin(Math.toRadians(currentPosition * 360));
    SmartDashboard.putNumber("Elbow Feed Forward", feedForward); 
    
    double forwardLimit = Constants.ElbowConstants.FORWARD_LIMIT;
    double reverseLimit = Constants.ElbowConstants.REVERSE_LIMIT;
    
    if(currentPosition >= forwardLimit && desiredSpeed > 0) {
      desiredSpeed = 0;
    } else if(currentPosition <= reverseLimit && desiredSpeed < 0) {
      desiredSpeed = 0;
    }
    
    motor.set(desiredSpeed + feedForward);
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    if(!RobotContainer.manual) {
      goToDesiredPosition();
    }
    SmartDashboard.putNumber("Elbow Desired Position", desiredPosition);
    SmartDashboard.putBoolean("Elbow at Setpoint", controller.atSetpoint());
    SmartDashboard.putNumber("Elbow Forward Limit", Constants.ElbowConstants.FORWARD_LIMIT);
    SmartDashboard.putNumber("Elbow Reverse Limit", Constants.ElbowConstants.REVERSE_LIMIT);
    SmartDashboard.putNumber("Elbow Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Elbow Motor Output", motor.get());
    SmartDashboard.putNumber("Elbow Position", canandmag.getPosition());
    SmartDashboard.putNumber("Elbow Supply Current", motor.getSupplyCurrent().getValueAsDouble());
  }
}
