package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;;

public class IntakePivot extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;
  private PositionVoltage positionVoltage;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private NeutralOut neutralOut;
  private double desiredPosition;

  public IntakePivot() {
    motor = new TalonFX(Constants.ID.INTAKE_PIVOT_ID);
    config = new TalonFXConfiguration();
    positionVoltage = new PositionVoltage(0).withSlot(0);
    neutralOut = new NeutralOut();
    canandmag = new Canandmag(Constants.ID.INTAKE_ENCODER_ID);
    canandmagSettings = new CanandmagSettings();
    canandmagSettings.getDisableZeroButton();
    canandmagSettings.setInvertDirection(false);

    config.Slot0.kP = .8;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.1;
    config.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    config.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivotConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.IntakePivotConstants.FORWARD_LIMIT*23.92268334280841;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.IntakePivotConstants.REVERSE_LIMIT*23.92268334280841;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    canandmag.setSettings(canandmagSettings);

    motor.setPosition(canandmag.getPosition()*23.92268334280841);
  }

  public void setPosition(double desiredPosition) {
    motor.setControl(
      positionVoltage
        .withPosition(desiredPosition*23.92268334280841)
    );
  }

  public void disengage() {
    motor.setControl(neutralOut);
  }

  public boolean isFinished() {
    return MathUtil.isNear(desiredPosition, motor.getPosition().getValueAsDouble(), Constants.IntakePivotConstants.TOLERANCE);
  }

  public void resetEncoder() {
    canandmag.setPosition(0);
    motor.setPosition(canandmag.getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakePivot/Setpoint", desiredPosition);
    SmartDashboard.putBoolean("IntakePivot/Is Finished", isFinished());
    SmartDashboard.putNumber("IntakePivot/Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("IntakePivot/Position", Double.parseDouble(String.format("%.2f", canandmag.getPosition())));
    SmartDashboard.putNumber("IntakePivot/Motor/Velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("IntakePivot/Motor/Applied Output", motor.get());
    SmartDashboard.putNumber("IntakePivot/Motor/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("IntakePivot/Motor/Position", motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("IntakePivot/Motor/Output Voltage", motor.getMotorVoltage().getValueAsDouble());
  }

}