// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollers extends SubsystemBase {

  private TalonFXS smallRoller;
  private TalonFXSConfiguration fxsConfig;
  private TalonFX bigRoller;
  private TalonFX indexerRoller;
  private TalonFXConfiguration fxConfig;

  public IntakeRollers() {

    smallRoller = new TalonFXS(Constants.ID.INTAKE_SMALL_ROLLER_ID);
    bigRoller = new TalonFX(Constants.ID.INTAKE_BIG_ROLLER_ID);
    indexerRoller = new TalonFX(Constants.ID.INTAKE_INDEXER_ID);

    fxsConfig = new TalonFXSConfiguration();

    fxsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    fxsConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakeRollerConstants.CURRENT_LIMIT;
    fxsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    fxsConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    smallRoller.getConfigurator().apply(fxsConfig);

    fxConfig = new TalonFXConfiguration();

    fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    fxConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakeRollerConstants.CURRENT_LIMIT;
    fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    bigRoller.getConfigurator().apply(fxConfig);
    indexerRoller.getConfigurator().apply(fxConfig);

  }

  public void setBigRollerSpeed(double DesiredSpeed) {
    bigRoller.set(-DesiredSpeed);
  }

  public void setSmallRollerSpeed(double desiredSpeed) {
    smallRoller.set(desiredSpeed);
  }

  public void setIndexerSpeed(double desiredSpeed) {
    indexerRoller.set(desiredSpeed);
  }

  public void stopAllMotors() {
    bigRoller.stopMotor();
    smallRoller.stopMotor();
    indexerRoller.stopMotor();
  }

  public void stopBigRoller() {
    bigRoller.stopMotor();
  }

  public void stopSmallRoller() {
    smallRoller.stopMotor();
  }

  public void stopIndexerRoller() {
    indexerRoller.stopMotor();
  }
 
  @Override
  public void periodic() {}
  
}
