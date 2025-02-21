package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Climber.ClimberDown;
import frc.robot.commands.Climber.ClimberUp;
import frc.robot.commands.EndEffector.EffectEnd;
import frc.robot.commands.Positioning.ElbowDown;
import frc.robot.commands.Positioning.ElbowUp;
import frc.robot.commands.Positioning.ElevatorDown;
import frc.robot.commands.Positioning.ElevatorUp;
import frc.robot.commands.Positioning.WristDown;
import frc.robot.commands.Positioning.WristUp;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    /* Controllers */
    private final CommandXboxController baseDriver = new CommandXboxController(Constants.ControlConstants.BASE_DRIVER_CONTROLLER_PORT);
    private final CommandXboxController armDriver = new CommandXboxController(Constants.ControlConstants.OPERATOR_DRIVER_CONTROLLER_PORT);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final Swerve Swerve = new Swerve();
    private final Wrist wrist = new Wrist();
    private final Elbow elbow = new Elbow();
    private final Elevator elevator = new Elevator();
    private final EndEffector endEffector = new EndEffector();
    private final Climber climber = new Climber();

    /* Commands */
    private final ClimberUp climberUp;
    private final ClimberDown climberDown;
    private final ElbowDown elbowDown;
    private final ElbowUp elbowUp;
    private final WristDown wristDown;
    private final WristUp wristUp;
    private final ElevatorDown elevatorDown;
    private final ElevatorUp elevatorUp;



    /* Autos */
    private final SendableChooser<Command> autoChooser;
    public static boolean isAlgae = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        Swerve.setDefaultCommand(
            new TeleopSwerve(
                Swerve, 
                () -> -baseDriver.getRawAxis(translationAxis), 
                () -> -baseDriver.getRawAxis(strafeAxis), 
                () -> -baseDriver.getRawAxis(rotationAxis), 
                () -> baseDriver.leftBumper().getAsBoolean()
            )
        );

        climberUp = new ClimberUp(climber);
        climberUp.addRequirements(climber);
        climberDown = new ClimberDown(climber);
        climberDown.addRequirements(climber);
        elbowDown = new ElbowDown(elbow);
        elbowDown.addRequirements(elbow);
        elbowUp = new ElbowUp(elbow);
        elbowUp.addRequirements(elbow);
        wristDown = new WristDown(wrist);
        wristDown.addRequirements(wrist);
        wristUp = new WristUp(wrist);
        wristUp.addRequirements(wrist);
        elevatorDown = new ElevatorDown(elevator);
        elevatorDown.addRequirements(elevator);
        elevatorUp = new ElevatorUp(elevator);
        elevatorUp.addRequirements(elevator);

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() { 
        /* zero robot heading when y is pressed on base driver controller */
        baseDriver.y().onTrue(new InstantCommand(() -> Swerve.zeroHeading()));
        baseDriver.leftTrigger().whileTrue(climberDown);
        baseDriver.rightTrigger().whileTrue(climberUp);

        /* TEMP/TESTING CONTROLS */
        armDriver.a().whileTrue(elevatorDown);
        armDriver.y().whileTrue(elevatorUp);
        armDriver.b().whileTrue(elbowUp);
        armDriver.x().whileTrue(elbowDown);

        armDriver.povUp().whileTrue(wristUp);
        armDriver.povDown().whileTrue(wristDown);

        armDriver.rightTrigger(.15).whileTrue(new EffectEnd(endEffector, true));
        armDriver.leftTrigger(.15).whileTrue(new EffectEnd(endEffector, false));

        /* FINAL CONTROLS */

        /* coral positions */
        //armDriver.a().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.l1CoralPosition));
        //armDriver.b().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.l2CoralPosition));
        //armDriver.y().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.l3CoralPosition));
        //armDriver.x().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.l4CoralPosition));

        /* intake */
        //armDriver.leftTrigger(.15).whileTrue(new EffectEnd(endEffector, true));
        //armDriver.rightTrigger(.15).whileTrue(new EffectEnd(endEffector, false));
        //armDriver.rightBumper().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.humanCoralIntakePosition));
        //armDriver.leftBumper().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.deepCagePosition));

        /* algae positions */
        //armDriver.povDown().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.groundAlgaeIntakePosition));
        //armDriver.povUp().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.processorPosition));
        //armDriver.povLeft().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.topAlgaePosition));
        //armDriver.povRight().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.bottomAlgaePosition));
        //armDriver.povUpLeft().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.algaeShootingPosition));
        //armDriver.povUpRight().onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.startPosition));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}

/*
 * Hey future T3 :) Before I leave I just wanted to thank 
 * this team for giving me the experiences it did.
 * I had no idea I was going to fall in love with making
 * things out of my own hands and watching them come to life.
 * But robotics showed me that, and I will forever be grateful
 * for it. I hope that as you guys code, build, and design
 * for this season you enjoy it as much as I always did. 
 * I know it's cheesy but please don't forget to stop 
 * and smell the roses. Remember why you are doing what you 
 * are doing. Even though things suck when stuff doesn't go
 * as planned, it's a part of the learning process and it 
 * makes you a smarter person as you adapt to it. I know we
 * can't always win and that's okay. You're learning. And that's
 * what matters. I don't think I can even imagine a high school 
 * experience without T3 in it. I devoted 4 years of my life to 
 * this crap and I have absolutely no regrets. As someone who has 
 * been programming for the team for the past few years I know
 * that it can be alot sometimes, but don't worry. Just do your
 * best with the knowledge you have. Also, I wouldn't have done
 * anything other than be the programmer for this team and I
 * loved that I did it. Thank you to all of the people that made
 * my robotics experiences so much better by just being in it.
 * Special shoutouts to: my favorite co-captains Anshu and Ryan,
 * Carrie my favorite coding partner, Anika the little sister
 * I've always wanted, and last but most certainly not least, 
 * Mr. Ware and Mr.Garren for enabling me to have such an
 * amazing educational experience with robotics. 
 * 
 * Thank you for everything,
 * Akshita Santra 
 */