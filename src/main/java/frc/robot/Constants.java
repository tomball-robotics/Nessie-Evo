package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final class ControlConstants {
        public static final int baseDriverControllerPort = 0;
        public static final int operatorDriverControllerPort = 1;
        public static final double stickDeadband = 0.1;
    }

    public static final class NotificationConstants {
        public static final double batteryLowVoltage = 11.5;
        public static final double batteryLowDuration = 5.0;
    }

    public static final class PositionConstants {
        /* 0. Elevator Position
         * 1. Elbow Position
         * 2. Wrist Angle
         * 3. Intake Speed -> "0" for coral, "1" for algae
         */
        public static final double[] startPosition = {0, 0, 0, 1}; //TODO tune
        public static final double[] l1CoralPosition = {10, 30, 15, 1}; //TODO tune
        public static final double[] l2CoralPosition = {20, 45, 25, 1}; //TODO tune
        public static final double[] l3CoralPosition = {30, 60, 35, 1}; //TODO tune
        public static final double[] l4CoralPosition = {40, 75, 45, 1}; //TODO tune
        public static final double[] bottomAlgaePosition = {5, 15, -10, 0}; //TODO tune
        public static final double[] topAlgaePosition = {35, 50, 40, 0}; //TODO tune
        public static final double[] humanCoralIntakePosition = {15, 20, 0, 1}; //TODO tune
        public static final double[] deepCagePosition = {22, 5, -25, 1}; //TODO tune
        public static final double[] processorPosition = {18, 35, 10, 0}; //TODO tune
        public static final double[] groundAlgaeIntakePosition = {22, 5, -25, 0}; //TODO tune
        public static final double[] algaeShootingPosition = {22, 5, -25, 1}; //TODO tune
    }

    public static final class EndEffectorConstants {
        public static final int endEffectorID = 0; //TODO tune
        public static final double endEffectorCurrentLimit = 30;
        public static final double coralIntakeSpeed = 0.5; //TODO tune
        public static final double coralOuttakeSpeed = -0.5; //TODO tune
        public static final double algaeIntakeSpeed = 0.5; //TODO tune
        public static final double algaeOuttakeSpeed = -0.5; //TODO tune
    }

    public static final class ElevatorConstants {
        public static final int elevatorMasterID = 0; //TODO tune
        public static final int elevatorSlaveID = 0; //TODO tune
        public static final int elevatorCurrentLimit = 30;
        public static final int canCoderID = 0; //TODO tune
        public static final double p = 1.6; //TODO tune
        public static final double i = 0.001; //TODO tune
        public static final double d = 0; //TODO tune
        public static final double tolerance = .01;
        public static final int forwardSoftLimit = 50; //TODO tune
        public static final int reverseSoftLimit = -50; //TODO tune
    }

    public static final class ClimberConstants {
        public static final int climberMotorID = 0; //TODO tune
        public static final double climberCurrentLimit = 30;
    }

    public static final class ElbowConstants{
        public static final int elbowMotorID = 0; //TODO tune
        public static final int elbowCurrentLimit = 30;
        public static final double p = 1.6; //TODO tune
        public static final double i = 0.001; //TODO tune
        public static final double d = 0; //TODO tune
        public static final double tolerance = .01;
        public static final int forwardSoftLimit = 50; //TODO tune
        public static final int reverseSoftLimit = -50; //TODO tune
    }

    public static final class WristConstants {
        public static final int wristMotorID = 12; //TODO tune
        public static final int wristCurrentLimit = 30;
        public static final double p = .001; //TODO tune
        public static final double i = 0.001; //TODO tune
        public static final double d = 0; //TODO tune
        public static final double tolerance = .01;
        public static final int forwardSoftLimit = 50; //TODO tune
        public static final int reverseSoftLimit = -50; //TODO tune
    }

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.LMatthew);
     
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.75);
        public static final double wheelBase = Units.inchesToMeters(24.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 10.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Drive Motor PID Values*/
        public static final double driveKP = 1;
        public static final double driveKI = 0.01;
        public static final double driveKD = 0.01;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.53114; //TODO tune
        public static final double driveKV = 2.3423; //TODO tune
        public static final double driveKA = 0.12817; //TODO tune

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO tune
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO tune

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 7; //TODO tune
            public static final int angleMotorID = 8; //TODO tune
            public static final int canCoderID = 4; //TODO tune
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-107.753906);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3; //TODO tune
            public static final int angleMotorID = 4; //TODO tune
            public static final int canCoderID = 2; //TODO tune
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(67.500000);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1; //TODO tune
            public static final int angleMotorID = 2; //TODO tune
            public static final int canCoderID = 1; //TODO tune
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(104.062500);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 5; //TODO tune
            public static final int angleMotorID = 6; //TODO tune
            public static final int canCoderID = 3; //TODO tune
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-60.468750);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3.85; //TODO tune
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.85; //TODO tune
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = .12; //TODO tune
        public static final double kPYController = 1; //TODO tune
        public static final double kPThetaController = 1; //TODO tune
    
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

}