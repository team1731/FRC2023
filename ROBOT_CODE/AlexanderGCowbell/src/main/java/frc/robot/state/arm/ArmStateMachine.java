package frc.robot.state.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.data.mp.ArmPath;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.ArmSubsystem;

public class ArmStateMachine {
  private ArmSubsystem subsystem;
  private ArmStatus status = ArmStatus.DISABLED;
  private GamePiece gamePiece = GamePiece.CUBE;
  private ArmState currentArmState = ArmState.HOME;
  private WristState currentWristState = WristState.HOME;
  private IntakeState currentIntakeState = IntakeState.STOPPED;
  private ArmPath currentPath;
  private MovementType movementType;
  private int currentPathIndex = 0;

  public enum MovementType {
    PICKUP, SCORE
  }

  public ArmStateMachine(ArmSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public ArmStatus getStatus() {
    return status;
  }

  public GamePiece getGamePiece() {
    return gamePiece;
  }

  public void setGamePiece(GamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }

  public MovementType getMovementType() {
    return movementType;
  }

  public void handle(ArmEvent event) {
    switch(event) {
      case BUTTON_DOWN:
        if(currentArmState == ArmState.PAUSED) {
          transitionArm(ArmInput.EXTEND);
        }
        break;
      case BUTTON_UP:
        if(currentArmState == ArmState.EXTENDING) {
          transitionArm(ArmInput.STOP);
        } else if(currentArmState == ArmState.EXTENDED) {
          if(movementType == MovementType.SCORE) {
            transitionIntake(ArmInput.RELEASE);
          }
          transitionArm(ArmInput.RETRACT);
        }
        break;
      case COMPLETED_PATH:
        transitionArm(ArmInput.COMPLETED);
        break;
      default:
        // do nothing
    }
  }
  
  public void pickup(ArmPath path) {
    if(path == null) return;
    if(!isReadyToStartMovement()) return; 

    System.out.println("ArmStateMachine: STARTING PICKUP!!!!!!!!!!!!!!!!!!!!!");
    currentPath = path;
    movementType = MovementType.PICKUP;
    transitionArm(ArmInput.EXTEND);
    transitionIntake(ArmInput.START);
  }

  public void score(ArmPath path) {
    if(path == null) return;
    if(!isReadyToStartMovement()) return; 

    System.out.println("ArmStateMachine: STARTING SCORE!!!!!!!!!!!!!!!!!!!!!");
    currentPath = path;
    movementType = MovementType.SCORE;
    transitionArm(ArmInput.EXTEND);
  }

  public void interrupt() {
    if(currentPath != null) {
      System.out.println("ArmStateMachine: INTERRUPTING!!!!!!!!!!!!!!!!!!!!!");
      transitionArm(ArmInput.INTERRUPT);
    }
  }

  private boolean isReadyToStartMovement() {
    if(currentArmState != ArmState.HOME ||
       currentPath != null) {
      return false;
    }
    return true;
  }

  public void periodic() {
    // Note: this periodic should be called each time the subsystem's periodic is called

  }

  private void transitionArm(ArmInput input) {
    ArmState prevState = currentArmState;
    currentArmState = currentArmState.next(input);
    if(currentArmState == prevState) {
      System.out.println("WARNING: arm state transition ignored, no change from " + prevState + ", input: " + input);
      return; 
    }

    System.out.println("ArmState Transition: " + input + " --> " + currentArmState);

    switch(currentArmState) {
      case EXTENDING:
        if(prevState == ArmState.HOME) {
          subsystem.startArmMovement(currentPath);
        } else {
          subsystem.restartArmMovement(currentPathIndex);
        }
        break;
      case PAUSED:
        subsystem.stopArm();
        break;
      case EXTENDED:
        // placeholder for possible check to allow extra extension
        break;
      case RETRACTING:
        subsystem.reverseArmMovment();
        break;
      case RESETTING_WRIST:
        subsystem.moveWristHome();
        break;
      case RESETTING_PROXIMAL:
        subsystem.moveProximalArmHome();
        break;
      case RESETTING_DISTAL:
        subsystem.moveDistalArmHome();
        break;
      case HOME:
        resetInternalState();
        break;
      default:
        System.out.println("WARNING: Invalid arm input sent to state machine: " + input + " --> " + currentArmState);
    }
  }

  private void transitionIntake(ArmInput input) {
    IntakeState prevState = currentIntakeState;
    currentIntakeState = currentIntakeState.next(input);
    if(currentIntakeState == prevState) {
      System.out.println("WARNING: intake state transition ignored, no change from " + prevState);
      return; 
    }

    System.out.println("IntakeState Transition: " + input + " --> " + currentIntakeState);

    switch(currentIntakeState) {
      case RETRIEVING:
        subsystem.intake();
        break;
      case HOLDING:
        subsystem.holdIntake();
        break;
      case RELEASING:
        subsystem.eject();
        break;
      case STOPPED:
        subsystem.stopIntake();
        break;
      default:
        System.out.println("WARNING: Invalid intake input sent to state machine: " + input + " --> " + currentIntakeState);
    }
  }

  private void resetInternalState() {
    status = ArmStatus.READY;
    currentPath = null;
    currentPathIndex = 0;
    movementType = null;
  }
}