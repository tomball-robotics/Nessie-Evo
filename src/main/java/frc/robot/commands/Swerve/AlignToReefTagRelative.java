// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private String desiredAlignment;
  private Timer dontSeeTagTimer, stopTimer;
  private Swerve swerve;
  private double tagID = -1;

  public AlignToReefTagRelative(String desiredAlignment, Swerve drivebase) {
    xController = new PIDController(Constants.AlignmentConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.AlignmentConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.AlignmentConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.desiredAlignment = desiredAlignment;
    this.swerve = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.AlignmentConstants.ROT_SETPOINT);
    rotController.setTolerance(Constants.AlignmentConstants.ROT_TOLERANCE_REEF_ALIGNMENT);
    xController.setTolerance(Constants.AlignmentConstants.X_TOLERANCE_REEF_ALIGNMENT);
    yController.setTolerance(Constants.AlignmentConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    switch(desiredAlignment) {
      case "left":
        xController.setSetpoint(Constants.AlignmentConstants.X_SETPOINT_CLOSE);
        yController.setSetpoint(Constants.AlignmentConstants.Y_SETPOINT_LEFT);
        break;
      case "right":
        xController.setSetpoint(Constants.AlignmentConstants.X_SETPOINT_CLOSE);
        yController.setSetpoint(Constants.AlignmentConstants.Y_SETPOINT_RIGHT);
        rotController.setSetpoint(2);
        break;
      case "center":
        xController.setSetpoint(Constants.AlignmentConstants.X_SETPOINT_CLOSE);
        yController.setSetpoint(Constants.AlignmentConstants.Y_SETPOINT_CENTER);
        break;
      case "back":
        xController.setSetpoint(Constants.AlignmentConstants.X_SETPOINT_FAR);
        yController.setSetpoint(Constants.AlignmentConstants.Y_SETPOINT_CENTER);
        break;
    }

    tagID = LimelightHelpers.getFiducialID("limelight-front");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-front") && LimelightHelpers.getFiducialID("limelight-front") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-front");
      SmartDashboard.putNumber("Alignment/x", postions[2]);
      SmartDashboard.putNumber("Alignment/y", postions[0]);
      SmartDashboard.putNumber("Alignment/rot", postions[4]);


      double xSpeed = xController.calculate(postions[2]);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      swerve.drive(
        new Translation2d(xSpeed, ySpeed).times(Constants.SwerveConstants.maxSpeed).times(.5), 
        rotValue * Constants.SwerveConstants.maxAngularVelocity, 
        false, 
        true
      );

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    }else {
      swerve.drive(new Translation2d(), 0, false, true);
    }

    SmartDashboard.putNumber("Alignment/poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false, true);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.AlignmentConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.AlignmentConstants.POSE_VALIDATION_TIME);
  }
}