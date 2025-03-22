package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NessieState;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.commands.position.SetElevatorPosition;
import frc.robot.commands.position.SetIntakePivotPosition;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;

public class StateMachine extends SubsystemBase {

  // nessie's states

  public static final NessieState START = new NessieState("Start", 0, 0, 0);
  public static final NessieState STOW = new NessieState( // done
    "Stow", .3, 0, 0);
  public static final NessieState L1 = new NessieState(
    "L1", 1, 0, 0);
  public static final NessieState L2 = new NessieState( // done
    "L2", 1.37799072265625, 0, 0);
  public static final NessieState L3 = new NessieState( // done
    "L3", 1.66351318359375, 0.86962890625, 0);
  public static final NessieState L4 = new NessieState( // done
    "L4", 1.85430908203125, 4.68536376953125, 0);
  public static final NessieState INTAKE = new NessieState(
    "Intake", 0, 0, 0);
  public static final NessieState ALGAE_TAXI = new NessieState(
    "Algae Stow", 0, 0, 0);
  public static final NessieState ALGAE_SHOOT = new NessieState(
    "Algae Shoot", 0, 0, 0);
  public static final NessieState ALGAE_INTAKE_LOW = new NessieState(
    "Algae Intake Low", 0, 0, 0);
  public static final NessieState ALGAE_INTAKE_HIGH = new NessieState(
    "Algae Intake High", 0, 0, 0);
  public static final NessieState ALGAE_PROCESS = new NessieState(
    "Algae Process", 0, 0, 0);

  private NessieState currentState;
  private double armClearancePosition = 0.437;

  private Elevator elevator;
  private Arm arm;
  private IntakePivot IntakePivot;

  public StateMachine(Elevator elevator, Arm arm, IntakePivot intakePivot, EndEffector endEffector) {
    this.elevator = elevator;
    this.arm = arm;
    this.IntakePivot = intakePivot;
   
    currentState = START;
    requestState(STOW);
  }

  public void requestState(NessieState desiredState) {

    if(currentState.getElevatorPosition() != desiredState.getElevatorPosition()) {
      new SetArmPosition(arm, armClearancePosition).execute();;
    }

    new SetElevatorPosition(elevator, desiredState.getElevatorPosition());
    new SetArmPosition(arm, desiredState.getArmPosition()).execute();

    new SetIntakePivotPosition(IntakePivot, desiredState.getIntakePosition());

    currentState = desiredState;
    
  }

  public boolean stateReached() {
    return elevator.atSetpoint() && arm.atSetpoint() && IntakePivot.atSetpoint();
  }

  public NessieState getCurrentState() {
    return currentState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("StateMachine/Elevated", getCurrentState().isElevated());
    SmartDashboard.putBoolean("StateMachine/Extended", getCurrentState().isExtended());
    SmartDashboard.putBoolean("StateMachine/Intake Extended", getCurrentState().isIntakeExtended());
    SmartDashboard.putString("StateMachine/Current State", getCurrentState().getName());
    SmartDashboard.putBoolean("StateMachine/State Reached", stateReached());
  }

}