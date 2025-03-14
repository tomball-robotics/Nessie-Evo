package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.AutoCoralIntake;
import frc.robot.commands.auto.AutoCoralOuttake;
import frc.robot.commands.climber.ClimberDown;
import frc.robot.commands.climber.ClimberUp;
import frc.robot.commands.endeffector.AlgaeIntake;
import frc.robot.commands.endeffector.AlgaeOuttake;
import frc.robot.commands.endeffector.CoralIntake;
import frc.robot.commands.endeffector.CoralOuttake;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(Constants.ControlConstants.DRIVER_PORT);
    private final CommandXboxController operator = new CommandXboxController(Constants.ControlConstants.OPERATOR_PORT);
    @SuppressWarnings("unused")
    private final GenericHID operatorKeypad = new GenericHID(Constants.ControlConstants.OPERATOR_KEYPAD_PORT);

    /* Drive Controls */
    private final int leftY = XboxController.Axis.kLeftY.value;
    private final int leftX = XboxController.Axis.kLeftX.value;
    private final int rightX = XboxController.Axis.kRightX.value;
    private final int rightY = XboxController.Axis.kRightY.value;

    /* Subsystems */
    private final Swerve swerve = new Swerve();
     private final Arm arm = new Arm(this);
    private final Elevator elevator = new Elevator(this);
    private final EndEffector endEffector = new EndEffector();
    private final Climber climber = new Climber();

    /* Commands */
    private final ClimberUp climberUp;
    private final ClimberDown climberDown;
    private final CoralIntake coralIntake;
    private final CoralOuttake coralOuttake;
    private final AlgaeIntake algaeIntake;
    private final AlgaeOuttake algaeOuttake;

    private final AutoCoralIntake autoCoralIntake;
    private final AutoCoralOuttake autoCoralOuttake;
    private final FastMode fastMode;
    private final SlowMode slowMode;

    /* Autos */
    private final SendableChooser<Command> autoChooser;

    /* Game */
    public boolean isAlgae = false;
    public boolean manual = false;

    public RobotContainer() {

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(leftY), 
                () -> -driver.getRawAxis(leftX), 
                () -> -driver.getRawAxis(rightX), 
                () -> false
            )
        );

        /* Climber Commands */

        climberUp = new ClimberUp(climber);
        climberUp.addRequirements(climber);
        climberDown = new ClimberDown(climber);
        climberDown.addRequirements(climber);

        /* Endeffector Commands */

        algaeIntake = new AlgaeIntake(endEffector);
        algaeIntake.addRequirements(endEffector);
        algaeOuttake = new AlgaeOuttake(endEffector);
        algaeOuttake.addRequirements(endEffector);
        coralIntake = new CoralIntake(endEffector);
        coralIntake.addRequirements(endEffector);
        coralOuttake = new CoralOuttake(endEffector);
        coralOuttake.addRequirements(endEffector);
        autoCoralIntake = new AutoCoralIntake(endEffector);
        autoCoralIntake.addRequirements(endEffector);
        autoCoralOuttake = new AutoCoralOuttake(endEffector);
        autoCoralOuttake.addRequirements(endEffector);

        /* Swerve Commands */
        fastMode = new FastMode(swerve);
        fastMode.addRequirements(swerve);
        slowMode = new SlowMode(swerve);
        slowMode.addRequirements(swerve);

        /* Register Commands & Autos */

        NamedCommands.registerCommand("Coral Intake", autoCoralIntake);
        NamedCommands.registerCommand("Coral Outtake", autoCoralOuttake);
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureButtonBindings();
    }

    private void configureButtonBindings() { 

        /* --- Driver --- */
        driver.y().onTrue(new InstantCommand(() -> swerve.zeroHeading()));
        driver.b().whileTrue(new InstantCommand(() -> swerve.setX()));
        driver.leftTrigger().whileTrue(climberDown);
        driver.rightTrigger().whileTrue(climberUp);
        driver.a().onTrue(fastMode);
        driver.b().onTrue(slowMode);

        /* --- Operator  --- */
        operator.rightBumper().whileTrue(coralOuttake);
        operator.leftBumper().whileTrue(coralIntake);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}