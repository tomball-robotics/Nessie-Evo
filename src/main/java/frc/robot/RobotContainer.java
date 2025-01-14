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


package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    /* Controllers */
    private final CommandXboxController baseDriver = new CommandXboxController(Constants.ControlConstants.baseDriverControllerPort);
    private final CommandXboxController armDriver = new CommandXboxController(Constants.ControlConstants.operatorDriverControllerPort);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /* Commands */
    private final SlowMode slowMode;
    private final FastMode fastMode;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -baseDriver.getRawAxis(translationAxis), 
                () -> -baseDriver.getRawAxis(strafeAxis), 
                () -> -baseDriver.getRawAxis(rotationAxis), 
                () -> baseDriver.leftBumper().getAsBoolean()
            )
        );

        slowMode = new SlowMode(s_Swerve);
        slowMode.addRequirements(s_Swerve);
        fastMode = new FastMode(s_Swerve);
        fastMode.addRequirements(s_Swerve);

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() { 
        /* zero robot heading when y is pressed on base driver controller */
        baseDriver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        /* press b to toggle between slowmode, 50% swerve power, and fastmode, 100% swerve power */
         baseDriver.b().onTrue(slowMode);
         baseDriver.b().onTrue(fastMode);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

}