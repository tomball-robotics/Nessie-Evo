package frc.robot;

import java.util.function.DoubleSupplier;

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
import frc.robot.commands.auto.AutoAlignToReefTagRelative;
import frc.robot.commands.auto.AutoCoralIntake;
import frc.robot.commands.auto.AutoCoralOuttake;
import frc.robot.commands.manual.ManualIntakeRollers;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.commands.position.SetElevatorPosition;
import frc.robot.commands.position.SetIntakePivotPosition;
import frc.robot.commands.manual.ManualEndEffector;
import frc.robot.commands.swerve.ChangeSpeedMultiplier;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.RevBlinkin;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;
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
    private final IntakePivot intakePivot = new IntakePivot();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final StateMachine stateMachine = new StateMachine(elevator, arm, intakePivot);

    /* Commands */
    private final AutoCoralIntake autoCoralIntake;
    private final AutoCoralOuttake autoCoralOuttake;
    private final ChangeSpeedMultiplier changeSpeedMultiplier;

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

        autoCoralIntake = new AutoCoralIntake(endEffector);
        autoCoralIntake.addRequirements(endEffector);
        autoCoralOuttake = new AutoCoralOuttake(endEffector);
        autoCoralOuttake.addRequirements(endEffector);
        changeSpeedMultiplier = new ChangeSpeedMultiplier(swerve);
        changeSpeedMultiplier.addRequirements(swerve);

        /* register commands & autos */
        NamedCommands.registerCommand("L1", new InstantCommand(() -> stateMachine.requestState(StateMachine.L1)));
        NamedCommands.registerCommand("L2", new InstantCommand(() -> stateMachine.requestState(StateMachine.L2)));
        NamedCommands.registerCommand("L3", new InstantCommand(() -> stateMachine.requestState(StateMachine.L3)));
        NamedCommands.registerCommand("L4", new InstantCommand(() -> stateMachine.requestState(StateMachine.L4)));
        NamedCommands.registerCommand("Algae Intake Low", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_LOW)));
        NamedCommands.registerCommand("Algae Intake Hiigh", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_HIGH)));
        NamedCommands.registerCommand("Algae Taxi", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_TAXI)));
        NamedCommands.registerCommand("Algae Shoot", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_SHOOT)));
        NamedCommands.registerCommand("Algae Process", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_PROCESS)));
        NamedCommands.registerCommand("Stow", new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)));
        NamedCommands.registerCommand("Align Right", new AutoAlignToReefTagRelative(true, swerve).withTimeout(3));
        NamedCommands.registerCommand("Align Left", new AutoAlignToReefTagRelative(false, swerve).withTimeout(3));
        NamedCommands.registerCommand("Shoot", autoCoralOuttake);
        NamedCommands.registerCommand("Intake", autoCoralIntake);

        SmartDashboard.putData("Zero Intake Pivot Encoder", new InstantCommand(() -> intakePivot.resetEncoder()));
        SmartDashboard.putData("Zero Elevator Encoder", new InstantCommand(() -> elevator.resetEncoder()));
        SmartDashboard.putData("Zero Arm Encoder", new InstantCommand(() -> arm.resetEncoder()));
        SmartDashboard.putData("Request Start State", new InstantCommand(() -> stateMachine.requestState(StateMachine.START)));
        SmartDashboard.putData("Request Disengaged State", new InstantCommand(() -> stateMachine.requestState(StateMachine.DISENGAGED)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);

        configureButtonBindings();

    }

    private void configureButtonBindings() {

        /* driver controls */

        driver.b().onTrue(changeSpeedMultiplier);

        driver.leftBumper().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.INTAKE)));
        driver.leftBumper().whileTrue(new ManualEndEffector(endEffector, Constants.EndEffectorConstants.CORAL_INTAKE_SPEED));
        driver.leftBumper().whileTrue(new ManualIntakeRollers(intakeRollers, .35, true));
        driver.leftBumper().onFalse(new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)));

        /* operator positions */

        operator.back().onTrue(new AutoAlignToReefTagRelative(false, swerve).withTimeout(3));
        operator.start().onTrue(new AutoAlignToReefTagRelative(true, swerve).withTimeout(3));

        operator.a().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.L1)));
        operator.b().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.L2)));
        operator.y().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.L3)));
        operator.x().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.L4)));

        operator.povUp().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_SHOOT)));
        operator.povRight().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_HIGH)));
        operator.povLeft().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_LOW)));
        operator.povDown().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_TAXI)));

        operator.leftBumper().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.INTAKE)));
        operator.leftBumper().whileTrue(new ManualIntakeRollers(intakeRollers, .2, false));
        operator.leftBumper().onFalse(new InstantCommand(() -> stateMachine.requestState(StateMachine.L1)).andThen(new WaitCommand(1)).andThen(new ManualIntakeRollers(intakeRollers, -.2, true)).withTimeout(2));

        operator.rightTrigger().onTrue(autoCoralOuttake);
        operator.leftTrigger().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
