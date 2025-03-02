package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private PIDController controller;
  private RobotContainer robotContainer;
  private double desiredPosition = 0;
  private double lastUpdateTime = 0;

  public Wrist(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    motor = new TalonFX(Constants.WristConstants.MOTOR_ID);
    config = new TalonFXConfiguration();
    canandmag = new Canandmag(Constants.WristConstants.ENCODER_ID);
    canandmagSettings = new CanandmagSettings();
    canandmagSettings.setInvertDirection(true);

    controller = new PIDController(
      Constants.WristConstants.P,
      Constants.WristConstants.I,
      Constants.WristConstants.D);
    controller.setTolerance(Constants.WristConstants.TOLERANCE);

    config.CurrentLimits.SupplyCurrentLimit = Constants.WristConstants.CURRENT_LIMIT;
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

    double forwardLimit = Constants.WristConstants.FORWARD_LIMIT;
    double reverseLimit = Constants.WristConstants.REVERSE_LIMIT;
    
    if(currentPosition >= forwardLimit && desiredSpeed > 0) {
      desiredSpeed = 0;
    } else if(currentPosition <= reverseLimit && desiredSpeed < 0) {
      desiredSpeed = 0;
    }
    
    motor.set(desiredSpeed);
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();
    if (!robotContainer.manual && (currentTime - lastUpdateTime >= Constants.ControlConstants.UPDATE_INTERVAL)) {
      lastUpdateTime = currentTime;
      goToDesiredPosition();
    }
    SmartDashboard.putNumber("Wrist Desired Position", desiredPosition);
    SmartDashboard.putBoolean("Wrist at Setpoint", controller.atSetpoint());
    SmartDashboard.putNumber("Wrist Forward Limit", Constants.WristConstants.FORWARD_LIMIT);
    SmartDashboard.putNumber("Wrist Reverse Limit", Constants.WristConstants.REVERSE_LIMIT);
    SmartDashboard.putNumber("Wrist Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Wrist Motor Output", motor.get());
    SmartDashboard.putNumber("Wrist Position", canandmag.getPosition());
    SmartDashboard.putNumber("Wrist Supply Current", motor.getSupplyCurrent().getValueAsDouble());
  }
}
