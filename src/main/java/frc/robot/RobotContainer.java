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
import frc.robot.commands.ManualElbow;
import frc.robot.commands.ManualElevator;
import frc.robot.commands.ManualEndEffector;
import frc.robot.commands.ManualWrist;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    /* Controllers */
    private final CommandXboxController baseDriver = new CommandXboxController(Constants.ControlConstants.BASE_DRIVER_CONTROLLER_PORT);
    private final CommandXboxController armDriver = new CommandXboxController(Constants.ControlConstants.OPERATOR_DRIVER_CONTROLLER_PORT);

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

    /* Autos */
    private final SendableChooser<Command> autoChooser;
    public static boolean isAlgae = false;

    public RobotContainer() {

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -baseDriver.getRawAxis(leftY), 
                () -> -baseDriver.getRawAxis(leftX), 
                () -> -baseDriver.getRawAxis(rightX), 
                () -> baseDriver.leftBumper().getAsBoolean()
            )
        );

        elevator.setDefaultCommand(
            new ManualElevator(
                elevator,
                () -> armDriver.getRawAxis(leftY)
            )
        );

        elbow.setDefaultCommand(
            new ManualElbow(
                elbow,
                () -> armDriver.getRawAxis(rightY)
            )
        );

        wrist.setDefaultCommand(
            new ManualWrist(
                wrist,
                () -> armDriver.getRawAxis(rightX)
            )
        );

        endEffector.setDefaultCommand(
            new ManualEndEffector(
                endEffector,
                () -> armDriver.getRawAxis(leftTrigger) - armDriver.getRawAxis(rightTrigger)
            )
        );

        climberUp = new ClimberUp(climber);
        climberUp.addRequirements(climber);
        climberDown = new ClimberDown(climber);
        climberDown.addRequirements(climber);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureButtonBindings();
    }

    private void configureButtonBindings() { 
        /* zero robot heading when y is pressed on base driver controller */
        baseDriver.y().onTrue(new InstantCommand(() -> swerve.zeroHeading()));
        baseDriver.leftTrigger().whileTrue(climberDown);
        baseDriver.rightTrigger().whileTrue(climberUp);

        //armDriver.a().onTrue(new InstantCommand(() -> elbow.setDesiredPosition(.3)));
       
    }

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