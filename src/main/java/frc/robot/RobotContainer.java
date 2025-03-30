package frc.robot;

import java.util.function.DoubleSupplier;

import javax.swing.JViewport;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoShootCoral;
import frc.robot.commands.ScoreLeft;
import frc.robot.commands.ScoreRight;
import frc.robot.commands.endeffector.IntakeAndHoldAlgae;
import frc.robot.commands.endeffector.IntakeAndHoldCoral;
import frc.robot.commands.endeffector.ShootAlgae;
import frc.robot.commands.endeffector.ShootCoral;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.commands.position.SetElevatorPosition;
import frc.robot.commands.swerve.AlignToReefTagRelative;
import frc.robot.commands.swerve.ChangeSpeedMultiplier;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.RevBlinkin;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;

@SuppressWarnings("unused")
public class RobotContainer {

    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.ControlConstants.DRIVER_PORT);
    public final CommandXboxController operator = new CommandXboxController(Constants.ControlConstants.OPERATOR_PORT);
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
    private final StateMachine stateMachine = new StateMachine(elevator, arm);

    /* Commands */
    private final IntakeAndHoldCoral intakeAndHoldCoral;
    private final IntakeAndHoldAlgae intakeAndHoldAlgae;
    private final ShootAlgae shootAlgae;
    private final ShootCoral shootCoral;
    private final AutoShootCoral autoShootCoral;
    private final ChangeSpeedMultiplier changeSpeedMultiplier;
    private final ScoreLeft scoreLeft;
    private final ScoreRight scoreRight;

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
                () -> driver.getHID().getRightBumperButton()
            )
        );

        intakeAndHoldAlgae = new IntakeAndHoldAlgae(endEffector);
        intakeAndHoldAlgae.addRequirements(endEffector);
        intakeAndHoldCoral = new IntakeAndHoldCoral(endEffector);
        intakeAndHoldCoral.addRequirements(endEffector);
        shootAlgae = new ShootAlgae(endEffector);
        shootAlgae.addRequirements(endEffector);
        shootCoral = new ShootCoral(endEffector);
        shootCoral.addRequirements(endEffector);
        autoShootCoral = new AutoShootCoral(endEffector);
        autoShootCoral.addRequirements(endEffector);
        scoreLeft = new ScoreLeft(stateMachine, arm, elevator, swerve, endEffector);
        scoreRight = new ScoreRight(stateMachine, arm, elevator, swerve, endEffector);
        changeSpeedMultiplier = new ChangeSpeedMultiplier(swerve);
        changeSpeedMultiplier.addRequirements(swerve);

        /* register commands & autos */
        NamedCommands.registerCommand("L1 Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.L1)));
        NamedCommands.registerCommand("L2 Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.L2)));
        NamedCommands.registerCommand("L3 Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.L3)));
        NamedCommands.registerCommand("L4 Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.L4)));
        NamedCommands.registerCommand("Algae Intake Low Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_LOW)));
        NamedCommands.registerCommand("Algae Intake High Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_HIGH)));
        NamedCommands.registerCommand("Algae Taxi Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_TAXI)));
        NamedCommands.registerCommand("Algae Shoot Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_SHOOT)));
        NamedCommands.registerCommand("Stow Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)));
        NamedCommands.registerCommand("Align Right", new AlignToReefTagRelative("right", swerve).withTimeout(3));
        NamedCommands.registerCommand("Align Left", new AlignToReefTagRelative("left", swerve).withTimeout(3));
        NamedCommands.registerCommand("Shoot Coral", autoShootCoral);

        //NamedCommands.registerCommand("Score Left L4",new InstantCommand(() -> stateMachine.setDesiredLevel(StateMachine.L4)).andThen(scoreLeft) );

        SmartDashboard.putData("Commands/Zero Elevator Encoder", new InstantCommand(() -> elevator.resetEncoder()));
        SmartDashboard.putData("Commands/Zero Arm Encoder", new InstantCommand(() -> arm.resetEncoder()));
        SmartDashboard.putData("Commands/Request Start State", new InstantCommand(() -> stateMachine.requestState(StateMachine.START)));
        SmartDashboard.putData("Commands/Request Disengaged State", new InstantCommand(() -> stateMachine.requestState(StateMachine.DISENGAGED)));

        SmartDashboard.putData("Commands/Request L1 Sate", new InstantCommand(() -> stateMachine.requestState(StateMachine.L1)));
        SmartDashboard.putData("Commands/Request L2 Sate", new InstantCommand(() -> stateMachine.requestState(StateMachine.L2)));
        SmartDashboard.putData("Commands/Request L3 Sate", new InstantCommand(() -> stateMachine.requestState(StateMachine.L3)));
        SmartDashboard.putData("Commands/Request L4 Sate", new InstantCommand(() -> stateMachine.requestState(StateMachine.L4)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);

        configureButtonBindings();

    }

    private void configureButtonBindings() {

        /* driver controls */

        driver.b().onTrue(changeSpeedMultiplier);

        driver.leftBumper().onTrue(
            new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW))
            .andThen(new WaitCommand(.2)).andThen(() -> stateMachine.requestState(StateMachine.INTAKE))
        );
        driver.leftBumper().whileTrue(intakeAndHoldCoral);
        driver.leftBumper().onFalse(
            new InstantCommand(() -> stateMachine.requestState(StateMachine.INTAKE_CLEARANCE))
            .andThen(new WaitCommand(.2)).andThen(() -> stateMachine.requestState(StateMachine.STOW))
        );

        driver.leftTrigger().onTrue(new AlignToReefTagRelative("left", swerve).withTimeout(3));
        driver.rightTrigger().onTrue(new AlignToReefTagRelative("right", swerve).withTimeout(3));

        driver.povUp().onTrue(new InstantCommand(() -> swerve.zeroHeading()));


        /* operator positions */

        // operator.a().onTrue(new InstantCommand(() -> stateMachine.setDesiredLevel(StateMachine.L1)));
        // operator.b().onTrue(new InstantCommand(() -> stateMachine.setDesiredLevel(StateMachine.L2)));
        // operator.y().onTrue(new InstantCommand(() -> stateMachine.setDesiredLevel(StateMachine.L3)));
        // operator.x().onTrue(new InstantCommand(() -> stateMachine.setDesiredLevel(StateMachine.L4)));

        operator.a().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.L1)));
        operator.b().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.L2)));
        operator.y().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.L3)));
        operator.x().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.L4)));


        operator.povUp().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_SHOOT)));
        operator.povRight().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_HIGH)));
        operator.povLeft().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_LOW)));
        operator.povDown().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_PROCESS)));

        operator.rightTrigger().whileTrue(shootCoral);

        operator.leftTrigger().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)));

        operator.leftBumper().whileTrue(intakeAndHoldAlgae);
        operator.rightBumper().whileTrue(shootAlgae);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
