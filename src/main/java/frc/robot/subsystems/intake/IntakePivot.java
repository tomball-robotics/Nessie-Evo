package frc.robot.subsystems.intake;

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

public class IntakePivot extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private PIDController controller;

  public IntakePivot() {
    motor = new TalonFX(Constants.ID.INTAKE_PIVOT_ID);
    config = new TalonFXConfiguration();
    canandmag = new Canandmag(Constants.ID.INTAKE_ENCODER_ID);
    canandmagSettings = new CanandmagSettings();
    canandmagSettings.setInvertDirection(true);

    controller = new PIDController(
      Constants.IntakePivotConstants.P,
      Constants.IntakePivotConstants.I,
      Constants.IntakePivotConstants.D);
    controller.setTolerance(Constants.IntakePivotConstants.TOLERANCE);

    config.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivotConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    canandmag.setSettings(canandmagSettings);
  }

  public void goTowardsDesiredPosition() {
    double currentPosition = canandmag.getPosition();
    double output = controller.calculate(currentPosition);
    setSpeed(output);
  }
  
  public void setSpeed(double desiredSpeed) {
    double currentPosition = canandmag.getPosition();
    double forwardLimit = Constants.IntakePivotConstants.FORWARD_LIMIT;
    double reverseLimit = Constants.IntakePivotConstants.REVERSE_LIMIT;

    if(currentPosition >= forwardLimit && desiredSpeed > 0) {
      desiredSpeed = 0;
    }else if(currentPosition <= reverseLimit && desiredSpeed < 0) {
      desiredSpeed = 0;
    }

    motor.set(desiredSpeed);
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
    SmartDashboard.putBoolean("IntakePivot/at Setpoint", controller.atSetpoint());
    SmartDashboard.putNumber("IntakePivot/Setpoint", controller.getSetpoint());
    SmartDashboard.putNumber("IntakePivot/Forward Limit", Constants.IntakePivotConstants.FORWARD_LIMIT);
    SmartDashboard.putNumber("IntakePivot/Reverse Limit", Constants.IntakePivotConstants.REVERSE_LIMIT);
    SmartDashboard.putNumber("IntakePivot/Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("IntakePivot/Position", canandmag.getPosition());
    SmartDashboard.putNumber("IntakePivot/Motor/Velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("IntakePivot/Motor/Applied Output", motor.get());
    SmartDashboard.putNumber("IntakePivot/Motor/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
  }

}