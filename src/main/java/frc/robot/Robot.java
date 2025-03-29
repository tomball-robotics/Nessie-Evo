// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.Elastic;
import frc.lib.Elastic.Notification;
import frc.lib.LimelightHelpers;
import frc.robot.subsystems.RevBlinkin;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command auto;
  private RobotContainer robotContainer;
  private double batteryLowStartTime = -1;
  public static RevBlinkin revBlinkin = new RevBlinkin();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    CanandEventLoop.getInstance();
    robotContainer = new RobotContainer();
    revBlinkin.redSolidColor();
    SmartDashboard.putBoolean("Align/Aligned", false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {       
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    /* elastic low voltage warning (<11.5v) */
    if (RobotController.getBatteryVoltage() < Constants.NotificationConstants.BATTERY_LOW_VOLTAGE) {
      if (batteryLowStartTime == -1) {
          batteryLowStartTime = Timer.getFPGATimestamp();
      }
      if (Timer.getFPGATimestamp() - batteryLowStartTime >= Constants.NotificationConstants.BATTERY_LOW_DURATION) {
          Elastic.sendNotification(new Notification(
              Elastic.Notification.NotificationLevel.WARNING,
              "Battery Voltage Low",
              "Battery Voltage is below " + 
              Constants.NotificationConstants.BATTERY_LOW_VOLTAGE + 
              "V for " + Constants.NotificationConstants.BATTERY_LOW_DURATION + " seconds"));
          batteryLowStartTime = -1;
      }
    }
    else {
        batteryLowStartTime = -1;
    }

    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    CommandScheduler.getInstance().run();
    double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-front");
    SmartDashboard.putNumber("Alignment/x", postions[2]);
    SmartDashboard.putNumber("Alignment/y", postions[0]);
    SmartDashboard.putNumber("Alignment/rot", postions[4]);
    SmartDashboard.putBoolean("Alignment/Valid Tag", LimelightHelpers.getTV("limelight-front"));
    if(LimelightHelpers.getTV("limelight-front")) {
      revBlinkin.strobeRedPattern();
    }else {
      revBlinkin.redSolidColor();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    revBlinkin.redSolidColor();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    auto = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (auto != null) {
      auto.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    revBlinkin.redSolidColor();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (auto != null) {
      auto.cancel();
    }
    new InstantCommand().schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}