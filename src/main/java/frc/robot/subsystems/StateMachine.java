package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NessieState;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.commands.position.SetElevatorPosition;
import frc.robot.commands.position.SetIntakePivotPosition;
import frc.robot.subsystems.intake.IntakePivot;

public class StateMachine extends SubsystemBase {

  // nessie's states

  public static final NessieState STOW = new NessieState(
    "Stow", 0, 0, 0);
  public static final NessieState L1 = new NessieState(
    "L1", 0, 0, 0);
  public static final NessieState L2 = new NessieState(
    "L2", 0, 0, 0);
  public static final NessieState L3 = new NessieState(
    "L3", 0, 0, 0);
  public static final NessieState L4 = new NessieState(
    "L4", 0, 0, 0);
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
  }

  public void requestState(NessieState desiredState) {

    new SetIntakePivotPosition(IntakePivot, desiredState.getIntakePosition());

     // if elevtor needs to run and the arm is gonna hit, give arm clearance

    if((currentState.getElevatorPosition() != desiredState.getElevatorPosition()) && !(currentState.getArmPosition() < armClearancePosition)) {
      new SetArmPosition(arm, armClearancePosition);
    }

    // run others
    new SetElevatorPosition(elevator, desiredState.getElevatorPosition());
    new SetArmPosition(arm, desiredState.getArmPosition());
    
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
  }

}