package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class Elevator extends SubsystemBase {

  private TalonFX motor;
  private TalonFX follower;
  private TalonFXConfiguration config;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private PIDController controller;
  private RobotContainer robotContainer;
  private double desiredPosition = 0;
  private double lastUpdateTime = 0;
  
  public Elevator(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    motor = new TalonFX(Constants.ElevatorConstants.MASTER_ID);
    follower = new TalonFX(Constants.ElevatorConstants.FOLLOWER_ID);
    canandmag = new Canandmag(Constants.ElevatorConstants.ENCODER_ID);
    canandmag.setPosition(0);

    controller = new PIDController(
      Constants.ElevatorConstants.P,
      Constants.ElevatorConstants.I,
      Constants.ElevatorConstants.D);
    controller.setTolerance(Constants.ElevatorConstants.TOLERANCE);

    config = new TalonFXConfiguration();

    canandmagSettings = new CanandmagSettings();

    canandmagSettings.setInvertDirection(true);

    config.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    canandmag.setSettings(canandmagSettings);

    motor.setNeutralMode(NeutralModeValue.Brake);
    follower.setNeutralMode(NeutralModeValue.Brake);

    follower.setControl(new Follower(10, false));
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
    double kS = Constants.ElevatorConstants.S; // static friction compensation

    double forwardLimit = Constants.ElevatorConstants.FORWARD_LIMIT;
    double reverseLimit = Constants.ElevatorConstants.REVERSE_LIMIT;

    if(currentPosition >= forwardLimit && desiredSpeed > 0) {
      desiredSpeed = 0;
    }else if(currentPosition <= reverseLimit && desiredSpeed < 0) {
      desiredSpeed = 0;
    }

    motor.set(desiredSpeed + kS);
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
    SmartDashboard.putNumber("Elevator Setpoint", controller.getSetpoint());
    SmartDashboard.putNumber("Elevator Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Elevator Desired Position", desiredPosition);
    SmartDashboard.putNumber("Elevator Motor Output", motor.get());
    SmartDashboard.putNumber("Elevator Position", canandmag.getPosition());
    SmartDashboard.putNumber("ELevator Supply Current", motor.getSupplyCurrent().getValueAsDouble());
  }
}
 