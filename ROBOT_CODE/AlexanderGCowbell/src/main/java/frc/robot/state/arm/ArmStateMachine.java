package frc.robot.state.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.data.mp.ArmPath;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.ArmSubsystem;

public class ArmStateMachine {
  private ArmSubsystem subsystem;
  private Status status = Status.READY;
  private GamePiece gamePiece = GamePiece.CUBE;
  private ArmState currentArmState = ArmState.HOME;
  private WristState currentWristState = WristState.HOME;
  private IntakeState currentIntakeState = IntakeState.STOPPED;
  private ArmPath currentPath;
  private MovementType movementType;
  private int currentPathIndex = 0;


  /*
   * CONSTANT VALUES
   */

  public enum Status {
    READY, RUNNING
  }

  public enum Input {
    EXTEND, COMPLETED, RETRACT, RESET, START, STOP, INTERRUPT,
    RETRIEVED, RELEASE, RELEASED, FLEX;
}

  public enum MovementType {
    PICKUP, SCORE
  }


  /*
   * INITIALIZATION/RESET
   */

  public ArmStateMachine(ArmSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  private void resetInternalState() {
    status = Status.READY;
    currentPath = null;
    currentPathIndex = 0;
    movementType = null;
  }


  /*
   * METHODS TO HANDLE PICKUP/SCORE/INTERRUPT
   */
  
  // PICKUP
  public void pickup(ArmPath path) {
    if(path == null) return;
    if(!isReadyToStartMovement()) return; 

    System.out.println("ArmStateMachine: STARTING PICKUP!!!!!!!!!!!!!!!!!!!!!");
    status = Status.RUNNING;
    currentPath = path;
    movementType = MovementType.PICKUP;
    transitionArm(Input.EXTEND);
    transitionIntake(Input.START);
  }

  // SCORE
  public void score(ArmPath path) {
    if(path == null) return;
    if(!isReadyToStartMovement()) return; 

    System.out.println("ArmStateMachine: STARTING SCORE!!!!!!!!!!!!!!!!!!!!!");
    status = Status.RUNNING;
    currentPath = path;
    movementType = MovementType.SCORE;
    transitionArm(Input.EXTEND);
  }

  // RESTART PICKUP OR SCORE
  public void restartMovement() {
    if(currentArmState == ArmState.PAUSED) {
      transitionArm(Input.EXTEND);
    }
  }

  // NOTIFY THAT PICKUP/SCORE BUTTON RELEASED
  public void buttonReleased() {
    if(currentArmState == ArmState.EXTENDING) {
      transitionArm(Input.STOP);
    } else if(currentArmState == ArmState.EXTENDED) {
      if(movementType == MovementType.SCORE) {
        transitionIntake(Input.RELEASE);
      }
      transitionArm(Input.RETRACT);
    }
  }

  private boolean isReadyToStartMovement() {
    if(currentArmState != ArmState.HOME ||
       currentPath != null) {
      return false;
    }
    return true;
  }


  /*
   * PERIODIC
   *  Note: this periodic should be called each time the subsystem's periodic is called
   */

  public void periodic() {

  }


  /*
   * STATE TRANSITIONS
   */

  private void transitionArm(Input input) {
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

  private void transitionIntake(Input input) {
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


  /*
   * GETTERS/SETTERS
   */

   public Status getStatus() {
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
}