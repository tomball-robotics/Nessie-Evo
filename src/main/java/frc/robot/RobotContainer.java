package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.AutoCoralIntake;
import frc.robot.commands.auto.AutoCoralOuttake;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.climber.ClimberDown;
import frc.robot.commands.climber.ClimberUp;
import frc.robot.commands.motions.ScoreL1;
import frc.robot.commands.motions.ScoreL2;
import frc.robot.commands.motions.ScoreL3;
import frc.robot.commands.motions.ScoreL4;
import frc.robot.commands.swerve.HoldAlign;
import frc.robot.commands.swerve.ChangeSpeedMultiplier;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.ControlConstants.DRIVER_PORT);
    public final CommandXboxController operator = new CommandXboxController(Constants.ControlConstants.OPERATOR_PORT);
    @SuppressWarnings("unused")
    private final GenericHID operatorKeypad = new GenericHID(Constants.ControlConstants.OPERATOR_KEYPAD_PORT);

    /* Drive Controls */
    private final int leftY = XboxController.Axis.kLeftY.value;
    private final int leftX = XboxController.Axis.kLeftX.value;
    private final int rightX = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final EndEffector endEffector = new EndEffector();
    private final Climber climber = new Climber();

    /* Commands */
    private final ClimberUp climberUp;
    private final ClimberDown climberDown;

    private final AutoCoralIntake autoCoralIntake;
    private final AutoCoralOuttake autoCoralOuttake;

    private final ScoreL1 scoreL1;
    private final ScoreL2 scoreL2;
    private final ScoreL3 scoreL3;
    private final ScoreL4 scoreL4;

    private final ChangeSpeedMultiplier changeSpeedMultiplier;
    private final HoldAlign holdAlign;
    private final AutoAlign pressAlign;

    /* Autos */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(leftY), 
                () -> -driver.getRawAxis(leftX), 
                () -> -driver.getRawAxis(rightX), 
                () -> driver.getHID().getXButton(),
                () -> false
            )
        );

        // arm.setDefaultCommand(
        //     new ManualArm(
        //         arm,
        //         () -> -operator.getLeftY()
        //     )
        // );

        // elevator.setDefaultCommand(
        //     new ManualElevator(
        //         elevator,
        //         () -> -operator.getRightY()
        //     )
        // );

        // climber
        climberUp = new ClimberUp(climber);
        climberUp.addRequirements(climber);
        climberDown = new ClimberDown(climber);
        climberDown.addRequirements(climber);

        // auto
        autoCoralIntake = new AutoCoralIntake(endEffector);
        autoCoralIntake.addRequirements(endEffector);
        autoCoralOuttake = new AutoCoralOuttake(endEffector);
        autoCoralOuttake.addRequirements(endEffector);

        // swerve
        changeSpeedMultiplier = new ChangeSpeedMultiplier(swerve);
        changeSpeedMultiplier.addRequirements(swerve);
        holdAlign = new HoldAlign(swerve, this);
        holdAlign.addRequirements(swerve);
        pressAlign = new AutoAlign(swerve);
        pressAlign.addRequirements(swerve);

        // superstructure
        scoreL1 = new ScoreL1(arm, endEffector);
        scoreL1.addRequirements(arm, endEffector);
        scoreL2 = new ScoreL2(arm, elevator, endEffector);
        scoreL2.addRequirements(arm, elevator, endEffector);
        scoreL3 = new ScoreL3(arm, elevator, endEffector);
        scoreL3.addRequirements(arm, elevator, endEffector);
        scoreL4 = new ScoreL4(arm, elevator, endEffector);
        scoreL4.addRequirements(arm, elevator, endEffector);

        // register commands & autos
        NamedCommands.registerCommand("ScoreL1", scoreL1);
        NamedCommands.registerCommand("ScoreL2", scoreL2);
        NamedCommands.registerCommand("ScoreL3", scoreL3);
        NamedCommands.registerCommand("ScoreL4", scoreL4);
        NamedCommands.registerCommand("AlignRight", null);
        NamedCommands.registerCommand("AlignLeft", null);
        NamedCommands.registerCommand("AlignCenter", null);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        // driver
        driver.b().onTrue(changeSpeedMultiplier);

        // operator
        // operator.rightBumper().onTrue(new InstantCommand(() -> swerve.setDesiredAlignment("right")));
        // operator.rightBumper().whileTrue(align);

        // operator.leftBumper().onTrue(new InstantCommand(() -> swerve.setDesiredAlignment("left")));
        // operator.leftBumper().whileTrue(align);

        // operator.povDown().onTrue(new InstantCommand(() -> swerve.setDesiredAlignment("center")));
        // operator.povDown().whileTrue(align);

        operator.a().onTrue(scoreL1);
        operator.b().onTrue(scoreL2);
        operator.x().onTrue(scoreL3);
        operator.y().onTrue(scoreL4);

        operator.a().onTrue(autoCoralIntake);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
