package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NessieState;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.commands.position.SetElevatorPosition;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.subsystems.superstructure.Elevator;

public class StateMachine extends SubsystemBase {

  public static final NessieState START = new NessieState(
    "Start", 0, 0, 0);
  public static final NessieState STOW = new NessieState( // done
    "Stow", 4.5, 0, 0);
  public static final NessieState L1 = new NessieState(
    "L1", 2.7, 0, .08);
  public static final NessieState L2 = new NessieState( // done
    "L2", 12.40191650390625, 0, 0);
  public static final NessieState L3 = new NessieState( // done
    "L3", 15.51161865234375, 7.82666015625, 0);
  public static final NessieState L4 = new NessieState( // done
    "L4", 15.3, 50, 0);
  public static final NessieState INTAKE = new NessieState(
    "Intake", 1.08, 0, 0);
  public static final NessieState INTAKE_CLEARANCE = new NessieState(
    "Intake Clearance", 1.08, 16.74, 0);
  public static final NessieState ALGAE_TAXI = new NessieState(
    "Algae Stow", 17.37, 0, 0);
  public static final NessieState ALGAE_SHOOT = new NessieState(
    "Algae Shoot", 15.3, 50, 0);
  public static final NessieState ALGAE_INTAKE_LOW = new NessieState(
    "Algae Intake Low", 7.47, 0, 0);
  public static final NessieState ALGAE_INTAKE_HIGH = new NessieState(
    "Algae Intake High", 9.09, 12.69, 0);
  public static final NessieState ALGAE_PROCESS = new NessieState(
    "Algae Process", 0, 0, 0);
  public static final NessieState DISENGAGED = new NessieState(
    "Disengaged", -1, -1, -1);

  private NessieState currentState;
  public NessieState desiredLevel = L4;

  private Elevator elevator;
  private Arm arm;

  public StateMachine(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;

    currentState = DISENGAGED;
  }

  public void setDesiredLevel(NessieState desiredLevel) {
    this.desiredLevel = desiredLevel;
  }

  public NessieState getDesiredLevel() {
    return desiredLevel;
  }

  public void requestState(NessieState desiredState) {
    if(desiredState == DISENGAGED) {
      new InstantCommand(() -> arm.disengage()).schedule();
      new InstantCommand(() -> elevator.disengage()).schedule();
    }else {
      new SetArmPosition(arm, desiredState.getArmPosition()).schedule();
      new SetElevatorPosition(elevator, desiredState.getElevatorPosition()).schedule();
    }
    
    currentState = desiredState;
  }

  public boolean stateReached() {
    return elevator.isFinished() && arm.isFinished();
  }

  public NessieState getCurrentState() {
    return currentState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("StateMachine/Current State", currentState.getName());
    SmartDashboard.putBoolean("StateMachine/State Reached", stateReached());
    SmartDashboard.putString("StateMachine/Desired Level", desiredLevel.getName());
  }

}
