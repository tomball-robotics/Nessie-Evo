package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SetPosition;
import frc.robot.commands.SwapControlMode;
import frc.robot.commands.Climber.ClimberDown;
import frc.robot.commands.Climber.ClimberUp;
import frc.robot.commands.EndEffector.AlgaeIntake;
import frc.robot.commands.EndEffector.AlgaeOuttake;
import frc.robot.commands.EndEffector.CoralIntake;
import frc.robot.commands.EndEffector.CoralOuttake;
import frc.robot.commands.Manual.ManualElbow;
import frc.robot.commands.Manual.ManualElevator;
import frc.robot.commands.Manual.ManualEndEffector;
import frc.robot.commands.Manual.ManualWrist;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(Constants.ControlConstants.DRIVER_PORT);
    private final CommandXboxController operator = new CommandXboxController(Constants.ControlConstants.OPERATOR_PORT);
    private final GenericHID operatorKeypad = new GenericHID(Constants.ControlConstants.OPERATOR_KEYPAD_PORT);

    /* Drive Controls */
    private final int leftY = XboxController.Axis.kLeftY.value;
    private final int leftX = XboxController.Axis.kLeftX.value;
    private final int rightX = XboxController.Axis.kRightX.value;
    private final int rightTrigger = XboxController.Axis.kRightTrigger.value;
    private final int leftTrigger = XboxController.Axis.kLeftTrigger.value;
    private final int rightY = XboxController.Axis.kRightY.value;

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Wrist wrist = new Wrist();
    private final Elbow elbow = new Elbow();
    private final Elevator elevator = new Elevator();
    private final EndEffector endEffector = new EndEffector();
    private final Climber climber = new Climber();

    /* Commands */
    private final ClimberUp climberUp;
    private final ClimberDown climberDown;
    private final CoralIntake coralIntake;
    private final CoralOuttake coralOuttake;
    private final AlgaeIntake algaeIntake;
    private final AlgaeOuttake algaeOuttake;
    private final SwapControlMode swapControlMode;

    /* Autos */

    /* Game */
    public static boolean isAlgae = false;
    public static boolean manual = true;

    public RobotContainer() {

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(leftY), 
                () -> -driver.getRawAxis(leftX), 
                () -> -driver.getRawAxis(rightX), 
                () -> driver.leftBumper().getAsBoolean()
            )
        );

        if(RobotContainer.manual) {

            elevator.setDefaultCommand(
                new ManualElevator(
                    elevator,
                    () -> -(operator.getRawAxis(leftTrigger) - operator.getRawAxis(rightTrigger))
                )
            );

            elbow.setDefaultCommand(
                new ManualElbow(
                    elbow,
                    () -> operator.getRawAxis(rightY)
                )
            );

            wrist.setDefaultCommand(
                new ManualWrist(
                    wrist,
                    () -> -operator.getRawAxis(leftY)
                )
            );

            endEffector.setDefaultCommand(
                new ManualEndEffector(
                    endEffector,
                    () -> operator.getRawAxis(leftX)
                )
            );
        }

        climberUp = new ClimberUp(climber);
        climberUp.addRequirements(climber);
        climberDown = new ClimberDown(climber);
        climberDown.addRequirements(climber);
        algaeIntake = new AlgaeIntake(endEffector);
        algaeIntake.addRequirements(endEffector);
        algaeOuttake = new AlgaeOuttake(endEffector);
        algaeOuttake.addRequirements(endEffector);
        coralIntake = new CoralIntake(endEffector);
        coralIntake.addRequirements(endEffector);
        coralOuttake = new CoralOuttake(endEffector);
        coralOuttake.addRequirements(endEffector);
        swapControlMode = new SwapControlMode();

        configureButtonBindings();
    }

    private void configureButtonBindings() { 
        /* --- Driver --- */
        driver.y().onTrue(new InstantCommand(() -> swerve.zeroHeading()));
        driver.leftTrigger().whileTrue(climberDown);
        driver.rightTrigger().whileTrue(climberUp);

        /* --- Operator  --- */
        operator.a().whileTrue(coralIntake);
        operator.b().whileTrue(coralOuttake);
        operator.y().whileTrue(algaeOuttake);
        operator.x().whileTrue(algaeIntake);

        /* --- Operator Keypad  --- */
        new JoystickButton(operatorKeypad, 1).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.L1_CORAL_POSITION)); // top left
        new JoystickButton(operatorKeypad, 2).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.L2_CORAL_POSITION));
        new JoystickButton(operatorKeypad, 3).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.L3_CORAL_POSITION));
        new JoystickButton(operatorKeypad, 4).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.L4_CORAL_POSITION));
        new JoystickButton(operatorKeypad, 5).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.GROUND_ALGAE_POSITION));
        new JoystickButton(operatorKeypad, 6).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.DEEP_CAGE_POSITION)); // bottom left
        new JoystickButton(operatorKeypad, 7).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.STOW_POSITION)); // top right
        new JoystickButton(operatorKeypad, 8).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.TOP_ALGAE_POSITION));
        new JoystickButton(operatorKeypad, 9).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.BOTTOM_ALGAE_HARVEST));
        new JoystickButton(operatorKeypad, 10).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.PROCESSOR_POSITION));
        new JoystickButton(operatorKeypad, 11).onTrue(new SetPosition(elevator, elbow, wrist, Constants.PositionConstants.HUMAN_CORAL_POSITION));
        new JoystickButton(operatorKeypad, 12).onTrue(swapControlMode); // bottom right


    }

    public Command getAutonomousCommand() {
        return null;
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