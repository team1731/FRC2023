package frc.robot.state.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.data.mp.ArmPath;
import frc.data.mp.ArmPath.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.ArmSubsystem;

public class ArmStateMachine {
  private ArmSubsystem subsystem;
  private Status status = Status.READY;
  private GamePiece gamePiece = GamePiece.CUBE;
  private MovementType movementType;
  private boolean isInAuto = false;

  private ArmState currentArmState = ArmState.HOME;
  private IntakeState currentIntakeState = IntakeState.STOPPED;
  private boolean allowScore = true;

  private ArmPath currentPath;
  private int pathStartedIndex = 0;
  private double pathStartedTime = 0;

  // extended = home pos (palm facing out), flexed = positioned for pickup/scoring (palm facing down/in)
  private boolean wristFlexed = false; 


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

  public void resetState() {
    status = Status.READY;
    currentArmState = ArmState.HOME;
    currentIntakeState = IntakeState.STOPPED;
    currentPath = null;
    pathStartedIndex = 0;
    pathStartedTime = 0;
    movementType = null;
    wristFlexed = false;
    isInAuto = false;
  }


  /*
   * METHODS TO HANDLE PICKUP/SCORE
   */
  
  // PICKUP
  public void pickup(ArmPath path) {
    if(path == null) return;
    if(!isReadyToStartMovement()) return; 

    System.out.println("ArmStateMachine: STARTING PICKUP!!!!!!!!!!!!!!!!!!!!!");
    currentPath = path;
    pathStartedIndex = 0;
    movementType = MovementType.PICKUP;
    transitionArm(Input.EXTEND);
    transitionIntake(Input.START);
  }

  // SCORE
  public void score(ArmPath path) {
    if(path == null) return;
    if(!isReadyToStartMovement()) return; 

    System.out.println("ArmStateMachine: STARTING SCORE!!!!!!!!!!!!!!!!!!!!!");
    currentPath = path;
    pathStartedIndex = 0;
    movementType = MovementType.SCORE;
    transitionArm(Input.EXTEND);
  }

  // NOTIFY THAT PICKUP/SCORE BUTTON RELEASED
  public void buttonReleased() {
    if(currentArmState == ArmState.EXTENDING) {
      interrupt();
    } else if(currentArmState == ArmState.EXTENDED) {
      if(movementType == MovementType.SCORE && allowScore) {
        transitionIntake(Input.RELEASE);
      }
      transitionArm(Input.RETRACT);
    }
  }

  // NOTIFY THAT SUBSYSTEM COMPLETED ARM MOVEMENT
  public void completedArmMovement() {
    transitionArm(Input.COMPLETED);
  }

  // NOTIFY THAT PICKUP/SCORE SHOULD BE STOPPED & RETRACTED W/O SCORE/PICKUP
  public void interrupt() {
    subsystem.stopArm();
    transitionArm(Input.INTERRUPT);
    if(currentIntakeState == IntakeState.RETRIEVING || currentIntakeState == IntakeState.RELEASING) {
      transitionIntake(Input.STOP);
    }
  }

  private boolean isReadyToStartMovement() {
    if(status != Status.READY ||
       currentArmState != ArmState.HOME ||
       currentPath != null) {
      return false;
    }
    return true;
  }


  /*
   * METHODS TO HANDLE IMMEDIATE INTAKE/EJECT
   * These methods are intended to be called only by operator buttons
   */

  public void intake() {
    transitionIntake(Input.START);
  }

  public void stopIntake() {
    transitionIntake(Input.STOP);
  }

  public void release() {
    transitionIntake(Input.RELEASE);
  }

  public void stopRelease() {
    transitionIntake(Input.STOP);
  }


  /*
   * PATH TRACKING
   */

  public void startedPath() {
    status = Status.RUNNING;
    pathStartedTime = Timer.getFPGATimestamp();
  }

  public int getPathIndex() {
    Direction direction = subsystem.getDirection();
    int pointsLastIndex = currentPath.getNumberOfPoints()-1;
    double elapsedTimeMS = (Timer.getFPGATimestamp() - pathStartedTime) * 1000;
    int pointsProcessed = (int)(elapsedTimeMS / ArmConstants.pointDurationMS);
    int position = (direction == Direction.FORWARD)? pathStartedIndex + pointsProcessed : pathStartedIndex - pointsProcessed;
    // make sure we don't end up with an array out of bounds exception
    if(position > pointsLastIndex) {
        return pointsLastIndex;
    } else if (position < 0) {
        return 0;
    }

    return position;
  }


  /*
   * PERIODIC
   *  Note: this periodic should be called each time the subsystem's periodic is called
   */

  public void periodic() {

    /*
     * Logic for flexing/extending wrist
     */
    if(!wristFlexed && subsystem.isArmMoving() && subsystem.getDirection() == Direction.FORWARD) {
      int currentIndex = getPathIndex();
      int wristFlexIndex = currentPath.getWristFlexIndex();
      if(currentIndex >= wristFlexIndex) {
        // move the wrist into the flexed position for this path
        subsystem.moveWrist(currentPath.getWristFlexPosition(), currentPath.getWristMaxVelocity());
        wristFlexed = true;
      }
    } else if(wristFlexed && subsystem.isArmMoving() && subsystem.getDirection() == Direction.REVERSE) {
      int currentIndex = getPathIndex();
      int wristExtendIndex = currentPath.getWristExtendIndex();
      if(currentIndex <= wristExtendIndex) {
        // move the wrist back into extended (home) position
        subsystem.moveWrist(ArmConstants.wristHomePosition, currentPath.getWristMaxVelocity());
        wristFlexed = false;

        // by this point we will have released the game piece
        if(currentIntakeState == IntakeState.RELEASING) {
          transitionIntake(Input.RELEASED);
        }
      }
    }
    
    /*
     * Logic for moving intake into holding
     */
    if(currentIntakeState == IntakeState.RETRIEVING && subsystem.isIntakeAtHoldingVelocity()) {
      transitionIntake(Input.RETRIEVED);
    }

    
    /*
     * Logic for handling special cases for autonomous where we won't receive a button release event
     */
    if(isInAuto && movementType == MovementType.PICKUP && currentIntakeState == IntakeState.HOLDING) {
      transitionArm(Input.RETRACT);
    } else if(isInAuto && movementType == MovementType.SCORE && currentArmState == ArmState.EXTENDED) {
      transitionIntake(Input.RELEASE);
      transitionArm(Input.RETRACT);
    }
  }


  /*
   * STATE TRANSITIONS
   */

  private void transitionArm(Input input) {
    ArmState prevState = currentArmState;
    ArmState newState = currentArmState.next(input); 
    if(newState == prevState) {
      System.out.println("WARNING: arm state transition ignored, no change from " + prevState + ", input: " + input);
      return; 
    } else {
      currentArmState = newState;
    }

    System.out.println("ArmState Transition: " + input + " --> " + newState);

    switch(newState) {
      case EXTENDING:
        subsystem.startArmMovement(currentPath);
        break;
      case EXTENDED:
        // placeholder for possible check to allow extra extension
        break;
      case RETRACTING:
        // determine whether to start at current index (midstream) or if completed, at the end
        int startIndex = (subsystem.isArmMoving())? getPathIndex() : currentPath.getNumberOfPoints()-1;
        pathStartedIndex = startIndex;
        subsystem.reverseArmMovment(startIndex);
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
        resetState();
        break;
      default:
        System.out.println("WARNING: Invalid arm input sent to state machine: " + input + " --> " + newState);
    }
  }

  private void transitionIntake(Input input) {
    IntakeState prevState = currentIntakeState;
    IntakeState newState = currentIntakeState.next(input);
    if(newState == prevState) {
      System.out.println("WARNING: intake state transition ignored, no change from " + prevState);
      return; 
    } else {
      currentIntakeState = newState;
    }

    System.out.println("IntakeState Transition: " + input + " --> " + newState);

    switch(newState) {
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
        System.out.println("WARNING: Invalid intake input sent to state machine: " + input + " --> " + newState);
    }
  }


  /*
   * GETTERS/SETTERS
   */

  public Status getStatus() {
    return status;
  }

  public ArmState getArmState() {
    return currentArmState;
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

  public void setIsInAuto(boolean inAuto) {
    isInAuto = inAuto;
  }

  public void setAllowScore(boolean allow) {
    allowScore = allow;
  }
}