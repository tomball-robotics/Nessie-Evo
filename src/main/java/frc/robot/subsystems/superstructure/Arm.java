package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;;

public class Arm extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private PIDController controller;

  public Arm() {
    motor = new TalonFX(Constants.ID.ARM_TALONFX_ID);
    config = new TalonFXConfiguration();
    canandmag = new Canandmag(Constants.ID.ARM_ENCODER_ID);
    canandmagSettings = new CanandmagSettings();
    canandmagSettings.setInvertDirection(true);

    controller = new PIDController(
      Constants.ArmConstants.P,
      Constants.ArmConstants.I,
      Constants.ArmConstants.D);
    controller.setTolerance(Constants.ArmConstants.TOLERANCE);

    config.CurrentLimits.SupplyCurrentLimit = Constants.ArmConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    canandmag.setSettings(canandmagSettings);
    canandmag.setPosition(0);
  }

  private double feedforward(double position) {
    double halfPosition = 1.986328125; // this is what the position reads when the arm is straight up
    double kG = 0.013671875; // this is js the max feedforward, that theoeretically should be needed to hold the arm up in the .25 progression state

    double currentPosition = canandmag.getPosition();
    double progression = currentPosition/(halfPosition * 2);
    double progressionRads = progression * 2 *  Math.PI;
    double feedforward = kG * Math.sin(progressionRads);
    return feedforward;
  }

  public void goTowardsDesiredPosition() {
    double currentPosition = canandmag.getPosition();
    double output = controller.calculate(currentPosition);
    setSpeed(output);
  }

  public void setSpeed(double desiredSpeed) {
    double currentPosition = canandmag.getPosition();
    double forwardLimit = Constants.ArmConstants.FORWARD_LIMIT;
    double reverseLimit = Constants.ArmConstants.REVERSE_LIMIT;

    desiredSpeed = desiredSpeed + feedforward(currentPosition);

    if(currentPosition >= forwardLimit && desiredSpeed > 0) {
      desiredSpeed = 0;
    }else if(currentPosition <= reverseLimit && desiredSpeed < 0) {
      desiredSpeed = 0;
    }

    motor.set(desiredSpeed + feedforward(currentPosition));
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  public void setSetpoint(double desiredPosition) {
    controller.setSetpoint(desiredPosition);
  }

  @Override
  public void periodic() {
    //goTowardsDesiredPosition();
    SmartDashboard.putBoolean("Arm/at Setpoint", controller.atSetpoint());
    SmartDashboard.putNumber("Arm/Setpoint", controller.getSetpoint());
    SmartDashboard.putNumber("Arm/Forward Limit", Constants.ArmConstants.FORWARD_LIMIT);
    SmartDashboard.putNumber("Arm/Reverse Limit", Constants.ArmConstants.REVERSE_LIMIT);
    SmartDashboard.putNumber("Arm/Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Arm/Position", canandmag.getPosition());
    SmartDashboard.putNumber("Arm/Motor/Velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Motor/Applied Output", motor.get());
    SmartDashboard.putNumber("Arm/Motor/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
  }

}