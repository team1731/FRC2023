package frc.robot.state.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.data.mp.*;
import frc.data.mp.ArmPath.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmStateConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.ArmSubsystem;

public class ArmStateMachine {
  private ArmSubsystem subsystem;
  private Status status = Status.READY;
  private GamePiece gamePiece = GamePiece.CUBE;
  private MovementType movementType;
  private boolean isInAuto = false;
  private boolean isRunningKeypadEntry = false;
  private ArmSequence keyedSequence;

  private ArmState currentArmState = ArmState.UNKNOWN;
  private IntakeState currentIntakeState = IntakeState.STOPPED;
  private boolean allowScore = true;
  private boolean extraExtension = false;

  private ArmPath currentPath; // used if running full arm path
  private double currentWristFlexPosition = 0; // used if running wrist only movement
  private int pathStartedIndex = 0;
  private double pathStartedTime = 0;
  private QueuedCommand queuedCommand = null;
  private JoystickControl joystickControl = null;


  // extended = home pos (palm facing out), flexed = positioned for pickup/scoring (palm facing down/in)
  private boolean wristFlexed = false; 


  /*
   * CONSTANT VALUES
   */

  public enum Status {
    READY, RUNNING
  }

  public enum Input {
    INITIALIZE, EXTEND, COMPLETED, RETRACT, RESET, INTERRUPT,
    START, STARTED, STOP, DETECT_PIECE, RETRIEVED, RELEASE, RELEASED, 
    FLEX_WRIST, // flex for the wrist is palm down, e.g., when picking up/scoring
    EXTEND_WRIST; // extend for the wrist is palm out/up, e.g., when home
}

  public enum MovementType {
    PICKUP, SCORE
  }
  
  class QueuedCommand {
    public MovementType type;
    public ArmPath path; // used if running full arm path
    public double wristFlexPosition; // used if running wrist only movement
    public double queuedTime;

    public QueuedCommand(MovementType type, ArmPath path) {
      this.type = type;
      this.path = path;
      this.queuedTime = Timer.getFPGATimestamp();
    }

    public QueuedCommand(MovementType type, double wristFlexPosition) {
      this.type = type;
      this.wristFlexPosition = wristFlexPosition;
      this.queuedTime = Timer.getFPGATimestamp();
    }
  }

  class JoystickControl {
    public Joystick joystick;
    public int axis;
    public boolean enabled = false;
    public double startPosition = 0;

    public JoystickControl(Joystick joystick, int axis) {
      this.joystick = joystick;
      this.axis = axis;
    }

    public void setStartPosition(double startPosition) {
      this.startPosition = startPosition;
      enabled = true;
    }

    public double getRawAxis() {
      return startPosition + (joystick.getRawAxis(axis) * ArmConstants.distalMaxAdjustmentTicks);
    }
  }


  /*
   * INITIALIZATION/RESET
   */

  public ArmStateMachine(ArmSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public void resetState() {
    status = Status.READY;
    currentPath = null;
    currentWristFlexPosition = 0;
    pathStartedIndex = 0;
    pathStartedTime = 0;
    movementType = null;
    queuedCommand = null;
    joystickControl = null;
    wristFlexed = false;
    isInAuto = false;
    allowScore = true;
    extraExtension = false;

    if(isRunningKeypadEntry) {
      keyedSequence = null;
      isRunningKeypadEntry = false;
    }
  }

  // put arm into unknown state whenever disabled
  public void disabledInit() {
    System.out.println("ArmStateMachine: DISABLED!!!!!!!!!!!!!!!!!!!!!!!!!");
    resetState();
    currentArmState = ArmState.UNKNOWN;
  }

  // kick off a sequence to get us into our Home position safely
  public void initializeArm() {
    System.out.println("ArmStateMachine: INITIALIZING!!!!!!!!!!!!!!!!!!!!!");
    transitionArm(Input.INITIALIZE);
  }


  /*
   * METHODS TO HANDLE PICKUP/SCORE
   */
  
  // PICKUP
  public void pickup(ArmPath path) {
    if(path == null) return;
    if(!isReadyToStartMovement()) {
      if(isInAuto) {
        queuedCommand = new QueuedCommand(MovementType.PICKUP, path);
      }
      return;
    }

    System.out.println("ArmStateMachine: STARTING PICKUP!!!!!!!!!!!!!!!!!!!!!");
    currentPath = path;
    pathStartedIndex = 0;
    movementType = MovementType.PICKUP;
    transitionArm(Input.EXTEND);
    transitionIntake(Input.START);
  }

  // PICKUP WITH A WRIST FLEX ONLY
  public void pickup(double wristFlexPosition) {
    if(!isReadyToStartMovement()) {
      if(isInAuto) {
        queuedCommand = new QueuedCommand(MovementType.PICKUP, wristFlexPosition);
      }
      return;
    }

    System.out.println("ArmStateMachine: STARTING WRIST ONLY PICKUP!!!!!!!!!!!!!!!!!!!!!");
    currentWristFlexPosition = wristFlexPosition;
    movementType = MovementType.PICKUP;
    transitionArm(Input.FLEX_WRIST);
    transitionIntake(Input.START);
    status = Status.RUNNING;
  }

  // SCORE
  public void score(ArmPath path) {
    if(path == null) return;
    if(!isReadyToStartMovement()) {
      if(isInAuto) {
        queuedCommand = new QueuedCommand(MovementType.SCORE, path);
      }
      return;
    } 

    System.out.println("ArmStateMachine: STARTING SCORE!!!!!!!!!!!!!!!!!!!!!");
    currentPath = path;
    pathStartedIndex = 0;
    movementType = MovementType.SCORE;
    transitionArm(Input.EXTEND);
  }

  // SCORE BASED ON KEYPAD ENTRY
  public void scoreKeyedEntry() {
    if(!isReadyToStartMovement()) return;

    ArmPath path = null;
    if(keyedSequence == ArmSequence.SCORE_HIGH && gamePiece == GamePiece.CONE) {
      path = ScoreHighCone.getArmPath();
    } else if(keyedSequence == ArmSequence.SCORE_HIGH && gamePiece == GamePiece.CUBE) {
      path = ScoreHighCube.getArmPath();
    } else if(keyedSequence == ArmSequence.SCORE_MEDIUM && gamePiece == GamePiece.CONE) {
      path = ScoreMediumCone.getArmPath();
    } else if(keyedSequence == ArmSequence.SCORE_MEDIUM && gamePiece == GamePiece.CUBE) {
      path = ScoreMediumCube.getArmPath();
    } else if(keyedSequence == ArmSequence.SCORE_LOW && gamePiece == GamePiece.CONE) {
      path = ScoreLowCone.getArmPath();
    } else if(keyedSequence == ArmSequence.SCORE_LOW && gamePiece == GamePiece.CUBE) {
      path = ScoreLowCube.getArmPath();
    }
    
    if(path != null) {
      isRunningKeypadEntry = true;
      score(path);
    }
  }

  // NOTIFY THAT PICKUP/SCORE BUTTON RELEASED
  public void buttonReleased() {
    if(currentArmState == ArmState.EXTENDED || isMostlyExtended()) {
      if(movementType == MovementType.SCORE && allowScore) {
        transitionIntake(Input.RELEASE);
        transitionArm(Input.RETRACT);
      } else if(movementType == MovementType.PICKUP) {
        transitionIntake(Input.DETECT_PIECE); // note, will retract after completed detecting piece
      }
    } else if(currentArmState == ArmState.WRIST_ONLY_FLEXED && movementType == MovementType.PICKUP) {
      transitionIntake(Input.DETECT_PIECE); // note, will move wrist back after completed detecting piece
    } else if(currentArmState == ArmState.EXTENDING) {
      interrupt();
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
      System.out.println("WARNING: state machine failed on readiness check --> Status: " + status + 
        ", ArmState: " + currentArmState + 
        ", Path Already Loaded? " + (currentPath != null)
      );
      return false;
    }
    return true;
  }


  /*
   * METHODS TO HANDLE IMMEDIATE INTAKE/EJECT
   * These methods are intended to be called only by operator buttons
   */

  public void intake() {
    System.out.println("ArmStateMachine: Full intake requested!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    subsystem.intake();
  }

  public void releaseIntake() {
    System.out.println("ArmStateMachine: Releasing full intake!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    setIntakeHolding();
  }

  public void setIntakeHolding() {
    currentIntakeState = IntakeState.HOLDING;
    subsystem.holdIntake();
  }

  public void release() {
    System.out.println("ArmStateMachine: Full eject requested!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    subsystem.eject();
  }

  public void stopRelease() {
    System.out.println("ArmStateMachine: Releasing full eject!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    currentIntakeState = IntakeState.STOPPED;
    subsystem.stopIntake();
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

  private boolean isMostlyExtended() {
    if(currentArmState == ArmState.EXTENDING) {
      return ((getPathIndex() / currentPath.getNumberOfPoints()) >= ArmConstants.mostlyExtendedThreshold);
    }
    return false;
  }


  /*
   * PERIODIC
   *  Note: this periodic should be called each time the subsystem's periodic is called
   */

  public void periodic() {

    /*
     * Logic for handling queued auto commands
     * These commands can get queued when the command is requested while the arm is still reaching a ready state
     */
    if(isInAuto && queuedCommand != null) {
      if(isReadyToStartMovement() && queuedCommand.type == MovementType.PICKUP && queuedCommand.path != null) {
        pickup(queuedCommand.path);
        queuedCommand = null;
      } else if(isReadyToStartMovement() && queuedCommand.type == MovementType.PICKUP && queuedCommand.path == null) {
        pickup(queuedCommand.wristFlexPosition);
        queuedCommand = null;
      } else if(isReadyToStartMovement() && queuedCommand.type == MovementType.SCORE) {
        score(queuedCommand.path);
        queuedCommand = null;
      } else if(Timer.getFPGATimestamp() - queuedCommand.queuedTime > 2.0) {
        // something is wrong, we should have been home by now, clear the queued command
        queuedCommand = null;
      }
    } else if(!isInAuto && queuedCommand != null) {
      // we are no longer in auto, this command no longer applies, clear the queued command
      queuedCommand = null;
    }

    /*
     * Logic for flexing/extending wrist
     */
    if(!wristFlexed && subsystem.isMotionProfileRunning() && subsystem.getDirection() == Direction.FORWARD) {
      int currentIndex = getPathIndex();
      int wristFlexIndex = currentPath.getWristFlexIndex();
      if(currentIndex >= wristFlexIndex) {
        // move the wrist into the flexed position for this path
        subsystem.moveWrist(currentPath.getWristFlexPosition(), currentPath.getWristMaxVelocity());
        wristFlexed = true;
      }
    } else if(wristFlexed && subsystem.isMotionProfileRunning() && subsystem.getDirection() == Direction.REVERSE) {
      int currentIndex = getPathIndex();
      int wristExtendIndex = currentPath.getWristExtendIndex();
      if(currentIndex <= wristExtendIndex) {
        // move the wrist back into extended (home) position
        subsystem.moveWrist(ArmConstants.wristHomePosition, currentPath.getWristMaxVelocity());
        wristFlexed = false;
      }
    }
    
    /*
     * Logic for handling intake cases
     */
    if(currentIntakeState == IntakeState.STARTING && subsystem.isIntakeAtStartedVelocity()) {
      transitionIntake(Input.STARTED);
    }

    // currently only doing this in auto, in teleop retrieve continues until button release
    if(isInAuto && currentIntakeState == IntakeState.RETRIEVING && subsystem.isIntakeAtHoldingVelocity()) {
      transitionIntake(Input.RETRIEVED);
    }

    if(currentIntakeState == IntakeState.DETECTING_PIECE) {
      if(subsystem.isIntakeAtHoldingVelocity()) {
        // detected piece
        transitionIntake(Input.RETRIEVED);
      } else {
        transitionIntake(Input.STOP);
      }

      // deteced or failed to detect, regardless, move back to home
      if(currentArmState == ArmState.WRIST_ONLY_FLEXED) {
        transitionArm(Input.EXTEND_WRIST);
      } else {
        transitionArm(Input.RETRACT);
      }
    }

    if((currentIntakeState == IntakeState.RETRIEVING || currentIntakeState == IntakeState.RELEASING) && 
        currentArmState == ArmState.HOME) {
      // path has completed in reverse, make sure intake is stopped
      transitionIntake(Input.STOP);
    }
    
    /*
     * Logic for handling special cases for autonomous where we won't receive a button release event
     */
    if(isInAuto) {
      if(movementType == MovementType.PICKUP && currentIntakeState == IntakeState.HOLDING && currentArmState == ArmState.EXTENDED) {
        transitionArm(Input.RETRACT);
      } else if(movementType == MovementType.PICKUP && currentIntakeState == IntakeState.HOLDING && currentArmState == ArmState.WRIST_ONLY_FLEXED) {
        transitionArm(Input.EXTEND_WRIST);
      } else if(movementType == MovementType.SCORE && currentArmState == ArmState.EXTENDED) {
        transitionIntake(Input.RELEASE);
        transitionArm(Input.RETRACT);
      }
    }

    /*
     * Allow for joystick adjustment of the distal arm
     */
    if(currentArmState == ArmState.EXTENDED && joystickControl != null) {
      if(!joystickControl.enabled) {
        joystickControl.setStartPosition(subsystem.getDistalArmPosition());
      } else {
        subsystem.adjustDistalArm(joystickControl.getRawAxis());
      }
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
      case WRIST_ONLY_FLEXED:
        subsystem.moveWrist(currentWristFlexPosition, ArmStateConstants.wristOnlyFlexMaxVelocity);
        break;
      case RETRACTING:
        // determine whether to start at current index (midstream) or if completed, at the end
        int startIndex = (subsystem.isMotionProfileRunning())? getPathIndex() : currentPath.getNumberOfPoints()-1;
        pathStartedIndex = startIndex;
        subsystem.reverseArmMovment(startIndex);
        break;
      case RESETTING:
        subsystem.resetToHome();
        break;
      /*
      Temporarily disabling these states until we revise this process
      case RESETTING_WRIST:
        subsystem.moveWristHome();
        break;
      case RESETTING_PROXIMAL:
        subsystem.moveProximalArmHome();
        break;
      case RESETTING_DISTAL:
        subsystem.moveDistalArmHome();
        break;
      */
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
        // nothing to do, just waiting to detect that we are holding a piece
        break;
      case STARTING:
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
    if(gamePiece == null && status == Status.RUNNING) {
      return; // do not clear if running a path
    }
    this.gamePiece = gamePiece;
  }

  public boolean isHoldingGamePiece() {
    return (currentIntakeState == IntakeState.HOLDING);
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

  public boolean getExtraExtension() {
    return extraExtension;
  }

  public void setExtraExtension(boolean extraExtension) {
    this.extraExtension = extraExtension;
  }

  public void setKeyedSequence(String keypadEntry) {
    if(isRunningKeypadEntry) return; // do not allow keypad change if running a keyed path

    String[] keypadValues = keypadEntry.split("; ");
    if(keypadValues.length > 1) {
      String sequenceCode = keypadValues[1];
      keyedSequence = ArmSequence.valueForCode(sequenceCode);
    }
  }

  public ArmSequence getKeyedSequence() {
    return keyedSequence;
  }

  public void setJoystickControl(Joystick joystick, int axis) {
    joystickControl = new JoystickControl(joystick, axis);
  }
}