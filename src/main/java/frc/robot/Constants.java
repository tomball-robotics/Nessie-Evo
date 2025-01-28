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
        public static final double BATTERY_LOW_THRESHOLD = 11.5;
        public static final double BATTERY_LOW_DURATION = 5.0;
    }

    public static final class PositionConstants {
        // (elevator, elbow, wrist angles, algae or coal (1 for coral, 0 for algae))
        public static final double[] startPosition = {0, 0, 0, 1};
        public static final double[] l1CoralPosition = {10, 30, 15, 1};
        public static final double[] l2CoralPosition = {20, 45, 25, 1};
        public static final double[] l3CoralPosition = {30, 60, 35, 1};
        public static final double[] l4CoralPosition = {40, 75, 45, 1};
        public static final double[] bottomAlgaePosition = {5, 15, -10, 0};
        public static final double[] topAlgaePosition = {35, 50, 40, 0};
        public static final double[] humanCoralIntakePosition = {15, 20, 0, 1};
        public static final double[] deepCagePosition = {22, 5, -25, 1};
        public static final double[] processorPosition = {18, 35, 10, 0};
        public static final double[] groundAlgaeIntakePosition = {22, 5, -25, 0};
        public static final double[] algaeShootingPosition = {22, 5, -25, 1};
    }

    public static final class EndEffectorConstants {
        public static final int endEffectorID = 0; //TODO assign id
        public static final double coralIntakeSpeed = 0.5;
        public static final double coralOuttakeSpeed = -0.5;
        public static final double algaeIntakeSpeed = 0.5;
        public static final double algaeOuttakeSpeed = -0.5;
    }

    public static final class ElevatorConstants {
        public static final int elevatorMasterID = 0; //TODO assign id
        public static final int elevatorSlaveID = 0; //TODO assign id
        public static final int canCoderID = 0; //TODO assign id
        public static final double p = 1.6; 
        public static final double i = 0.001;
        public static final double d = 0;
        public static final double tolerance = .01;
        public static final int forwardSoftLimit = 50;
        public static final int reverseSoftLimit = -50;
        public static final int supplyCurrentLimit = 30;
    }

    public static final class ClimberConstants {
        public static final int climberMotorID = 0; //TODO assign id
    }

    public static final class ElbowConstants{
        public static final int elbowPivotID = 0; //TODO assign id
        public static final int elbowCancoderID = 0; //TODO assign id
        public static final double p = 1.6; 
        public static final double i = 0.001;
        public static final double d = 0;
        public static final double tolerance = .01;
        public static final int forwardSoftLimit = 50;
        public static final int reverseSoftLimit = -50;
        public static final int smartCurrentLimit = 30;
    }

    public static final class WristConstants {
        public static final int wristPivotID = 0; //TODO assign id
        public static final int wristCancoderID = 0; //TODO assign id
        public static final double p = 1.6; 
        public static final double i = 0.001;
        public static final double d = 0;
        public static final double tolerance = .01;
        public static final int forwardSoftLimit = 50;
        public static final int reverseSoftLimit = -50;
        public static final int smartCurrentLimit = 30;
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
        public static final double driveKS = 0.53114; // TODO tune
        public static final double driveKV = 2.3423;
        public static final double driveKA = 0.12817;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO tune
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-107.753906);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(67.500000);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(104.062500);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-60.468750);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3.85; //used to be 3
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.85;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = .12;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
         //Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

}