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
import frc.robot.Constants;;

public class Arm extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private PIDController controller;
  private double offset;
  private double desiredPosition = 0;
  private double lastUpdateTime = 0;

  public Arm() {
    motor = new TalonFX(10);
    config = new TalonFXConfiguration();
    canandmag = new Canandmag(16);
    canandmagSettings = new CanandmagSettings();
    offset = 0;
    canandmagSettings.setInvertDirection(true);

    controller = new PIDController(
      Constants.ArmConstants.P,
      Constants.ArmConstants.I,
      Constants.ArmConstants.D);
    controller.setTolerance(Constants.ArmConstants.TOLERANCE);

    config.CurrentLimits.SupplyCurrentLimit = Constants.ArmConstants.CURRENT_LIMIT;
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

  public void adjustOffset(double adjustment) {
    offset += adjustment;
  }

  private void goToDesiredPosition() {
    double currentPosition = canandmag.getPosition();
    double output = controller.calculate(currentPosition, (desiredPosition + offset));
    setSpeed(output);
  }

  public void setSpeed(double desiredSpeed) {
    double currentPosition = canandmag.getPosition();
    double feedForward = -0.04179303089 * Math.sin(Math.toRadians(currentPosition * 360));
    SmartDashboard.putNumber("Arm Feed Forward", feedForward); 
    
    double forwardLimit = Constants.ArmConstants.FORWARD_LIMIT;
    double reverseLimit = Constants.ArmConstants.REVERSE_LIMIT;
    
    if(currentPosition >= forwardLimit && desiredSpeed > 0) {
      desiredSpeed = 0;
    }else if(currentPosition <= reverseLimit && desiredSpeed < 0) {
      desiredSpeed = 0;
    }
    
    motor.set(desiredSpeed + feedForward);
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - lastUpdateTime >= Constants.ControlConstants.UPDATE_INTERVAL) {
      lastUpdateTime = currentTime;
      goToDesiredPosition();
    }
    SmartDashboard.putNumber("Arm Desired Position", desiredPosition);
    SmartDashboard.putBoolean("Arm at Setpoint", controller.atSetpoint());
    SmartDashboard.putNumber("Arm Forward Limit", Constants.ArmConstants.FORWARD_LIMIT);
    SmartDashboard.putNumber("Arm Reverse Limit", Constants.ArmConstants.REVERSE_LIMIT);
    SmartDashboard.putNumber("Arm Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Arm Motor Output", motor.get());
    SmartDashboard.putNumber("Arm Position", canandmag.getPosition());
    SmartDashboard.putNumber("Arm Supply Current", motor.getSupplyCurrent().getValueAsDouble());
  }
  
}
