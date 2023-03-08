package frc.robot.state.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.data.mp.*;
import frc.data.mp.ArmPath.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmStateConstants;
import frc.robot.Constants.OperatorConsoleConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.HighPickup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.log.LogWriter;

public class ArmStateMachine {
  private ArmSubsystem subsystem;
  private Status status = Status.READY;
  private GamePiece gamePiece = GamePiece.CONE;
  private HighPickup highPickup = HighPickup.FEEDER;
  private MovementType movementType;
  private boolean isInAuto = false;
  private boolean isRunningOperatorEntry = false;
  private ArmSequence operatorSequence = ArmSequence.SCORE_HIGH; // sequence pre-loaded by operator via keypad/switch

  private ArmState currentArmState = ArmState.UNKNOWN;
  private IntakeState currentIntakeState = IntakeState.STOPPED;
  private boolean allowScore = true;
  private boolean emergencyModeTriggeredNotConfirmed = false;

  private ArmPath currentPath; // used if running full arm path
  private double currentWristFlexPosition = 0; // used if running wrist only movement
  private int pathStartedIndex = 0;
  private double pathStartedTime = 0;
  private double wristMovementStartedTime = 0;
  private QueuedCommand queuedCommand = null;
  private JoystickControl proximalJoystickControl;
  private JoystickControl distalJoystickControl;


  // extended = home pos (palm facing out), flexed = positioned for pickup/scoring (palm facing down/in)
  private boolean wristFlexed = false; 


  /*
   * CONSTANT VALUES
   */

  public enum Status {
    READY, RUNNING, EMERGENCY_RECOVERY
  }

  public enum Input {
    INITIALIZE, EXTEND, COMPLETED, RETRACT, RESET, INTERRUPT,
    START, STARTED, STOP, DETECT_PIECE, RETRIEVED, RELEASE, RELEASED, 
    FLEX_WRIST, // flex for the wrist is palm down, e.g., when picking up/scoring
    AUTO_RECOVER; // attempts to take the bot out of emergency mode and recovery automatically
}

  public enum MovementType {
    PICKUP, PICKUP_DOWNED_CONE, SCORE
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
    public double startPosition = 0;
    // Note: distal joystick doubles for distal arm and wrist adjustment dependending on the path being run
    public boolean adjustWrist = false;

    public JoystickControl(Joystick joystick, int axis, boolean adjustWrist) {
      this.joystick = joystick;
      this.axis = axis;
      this.adjustWrist = adjustWrist;
    }

    public void setStartPosition(double startPosition) {
      this.startPosition = startPosition;
    }

    public double getPositionAdjustment() {
      if(adjustWrist){
        return startPosition + (joystick.getRawAxis(axis) * ArmConstants.wristMaxAdjustment);
      } else {
        return startPosition + (joystick.getRawAxis(axis) * ArmConstants.distalMaxAdjustmentTicks);
      }
    }

    public double getVelocityAdjustment() {
      return joystick.getRawAxis(axis) * ArmConstants.emergencyModeMaxArmVelocity;
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
    wristMovementStartedTime = 0;
    movementType = null;
    proximalJoystickControl = null;
    distalJoystickControl = null;
    wristFlexed = false;
    allowScore = true;
    emergencyModeTriggeredNotConfirmed = false;

    if(isRunningOperatorEntry) {
      operatorSequence = null;
      isRunningOperatorEntry = false;
    }
  }

  // put arm into unknown state whenever disabled
  public void disable() {
    System.out.println("ArmStateMachine: DISABLED!!!!!!!!!!!!!!!!!!!!!!!!!");
    resetState();
    currentArmState = ArmState.UNKNOWN;
    subsystem.allowArmManipulation();
  }

  // kick off a sequence to get the arm into the proper initial state
  public void initializeArm() {
    System.out.println("ArmStateMachine: INITIALIZING!!!!!!!!!!!!!!!!!!!!!");
    if(LogWriter.isArmRecordingEnabled()) {
      // ensure the arm and wrist motors so both can be moved freely for recording
      subsystem.allowArmManipulation();
    } else if(isArmOutOfPosition()) {
      status = Status.EMERGENCY_RECOVERY; // will still require operator to set kill switch
    } else {
      transitionIntake(Input.STOP);
      transitionArm(Input.INITIALIZE);
    }
  }

  // determine if the arm is out of safe position to be moved automatically to home
  public boolean isArmOutOfPosition() {
    return subsystem.getProximalArmPosition() < ArmConstants.proximalOutOfPositionThreshold;
  }


  /*
   * METHODS TO HANDLE RUN PATHS, PICKUP, SCORE
   */
  
  // PICKUP
  public void pickup(ArmPath path) {
    pickup(path, MovementType.PICKUP);
  }

  public void pickup(ArmPath path, MovementType movement) {
    if(path == null) return;
    if(!isReadyToStartMovement()) {
      if(isInAuto) {
        queuedCommand = new QueuedCommand(MovementType.PICKUP, path);
      }
      return;
    }

    System.out.println("ArmStateMachine: STARTING " + movement + "!!!!!!!!!!!!!!!!!!!!!");
    currentPath = path;
    pathStartedIndex = 0;
    movementType = movement;
    // make sure the intake is stopped before attempting to start it
    transitionIntake(Input.STOP); 
    transitionIntake(Input.START);
    // start the arm path
    transitionArm(Input.EXTEND);
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
    // make sure the intake is stopped before attempting to start it
    transitionIntake(Input.STOP);
    transitionIntake(Input.START);
    // move the wrist into position
    transitionArm(Input.FLEX_WRIST);
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

  // SCORE BASED ON OPERATOR ENTRY (KEYPAD OR SWITCH)
  public void scoreOperatorEntry() {
    if(!isReadyToStartMovement()) return;

    ArmPath path = null;
    if(operatorSequence == ArmSequence.SCORE_HIGH && gamePiece == GamePiece.CONE) {
      path = ScoreHighCone.getArmPath();
    } else if(operatorSequence == ArmSequence.SCORE_HIGH && gamePiece == GamePiece.CUBE) {
      path = ScoreHighCube.getArmPath();
    } else if(operatorSequence == ArmSequence.SCORE_MEDIUM && gamePiece == GamePiece.CONE) {
      path = ScoreMediumCone.getArmPath();
    } else if(operatorSequence == ArmSequence.SCORE_MEDIUM && gamePiece == GamePiece.CUBE) {
      path = ScoreMediumCube.getArmPath();
    } else if(operatorSequence == ArmSequence.SCORE_LOW && gamePiece == GamePiece.CONE) {
      path = ScoreLowCone.getArmPath();
    } else if(operatorSequence == ArmSequence.SCORE_LOW && gamePiece == GamePiece.CUBE) {
      path = ScoreLowCube.getArmPath();
    }
    
    if(path != null) {
      isRunningOperatorEntry = true;
      score(path);
    }
  }

  // NOTIFY THAT PICKUP/SCORE BUTTON RELEASED
  public void buttonReleased() {
    if(currentArmState == ArmState.EXTENDED || isMostlyExtended()) {
      if(movementType == MovementType.SCORE && allowScore) {
        transitionIntake(Input.RELEASE);
        initiateRetraction();
      } else if(movementType == MovementType.SCORE && !allowScore) {
        initiateRetraction();
      } else if(movementType == MovementType.PICKUP || movementType == MovementType.PICKUP_DOWNED_CONE) {
        initiateRetraction();
      }
    } else if(currentArmState == ArmState.WRIST_ONLY_FLEXED && movementType == MovementType.PICKUP) {
      transitionArm(Input.RETRACT);
      if(currentIntakeState != IntakeState.HOLDING) {
        transitionIntake(Input.STOP);
      }
    } else if(currentArmState == ArmState.EXTENDING) {
      interrupt();
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

  private void initiateRetraction() {
    if(currentArmState == ArmState.EXTENDED) {
      transitionArm(Input.RETRACT);
    } else {
      // we are initiating a midstream retraction
      subsystem.stopArm();
      transitionArm(Input.INTERRUPT);
    }
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

  public void clearCurrentPath() {
    if(currentArmState == ArmState.HOME) {
      resetState();
    } else {
      interrupt();
    }
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
      return ((double)getPathIndex() / currentPath.getNumberOfPoints()) >= ArmConstants.mostlyExtendedThreshold;
    }
    return false;
  }


  /*
   * INTERRUPTION/RECOVERY METHODS
   */

  // NOTIFY THAT PICKUP/SCORE SHOULD BE STOPPED & RETRACTED W/O SCORE/PICKUP
  public void interrupt() {
    initiateRetraction();
    if(currentIntakeState != IntakeState.HOLDING) {
      // movement was cut short, the intake should be stopped unless we're holding a piece
      transitionIntake(Input.STOP);
    }
  }

  // EITHER THE SYSTEM HAS DETECTED ITS OUT OF POSITION OR OPERATOR HAS HIT THE KILL SWITCH
  public void emergencyInterrupt() {
    if(!emergencyModeTriggeredNotConfirmed) {
      // the operator will need to confirm emergency mode before it will take effect
      emergencyModeTriggeredNotConfirmed = true; 
      return;
    }

    System.out.println("ArmStateMachine: Moved into EMERGENCY RECOVERY!!!!!!!!!!!!!!!!!!!!!!!!");
    emergencyModeTriggeredNotConfirmed = false; // it has been confirmed now
    subsystem.allowArmManipulation(); // arm and wrist will go limp
    subsystem.moveWristHome(); // snap the wrist home first to get it out of the way
    status = Status.EMERGENCY_RECOVERY;
    currentArmState = ArmState.EMERGENCY_RECOVERY;
  }

  // OPERATOR INDICATED ARM IS READY FOR AUTO RECOVERY ATTEMPT
  public void attemptAutoRecovery() {
    // ignore this if the arm is not acutally in recovery mode
    if(status == Status.EMERGENCY_RECOVERY) {
      System.out.println("ArmStateMachine: Attempting AUTO RECOVERY!!!!!!!!!!!!!!!!!!!!!!!!");
      transitionArm(Input.AUTO_RECOVER);
    }
  }


  /*
   * NOTIFICATIONS FROM SUBSYSTEM
   */

  // NOTIFY THAT SUBSYSTEM COMPLETED ARM MOVEMENT
  public void completedArmMovement() {
    transitionArm(Input.COMPLETED);
  }

  // NOTIFY THAT SUBSYSTEM COMPLETED ARM RETRACTION
  public void completedArmRetraction() {
    if(movementType == MovementType.PICKUP_DOWNED_CONE && currentIntakeState != IntakeState.HOLDING) {
      if(subsystem.isIntakeAtHoldingVelocity()) {
        // looks like we got it
        transitionIntake(Input.RETRIEVED); 
      } else {
        // didn't get the piece, stop the intake
        transitionIntake(Input.STOP);
      }
    } else if(currentIntakeState != IntakeState.HOLDING) {
      // in most scenarios, upon retraction, the intake should be stopped unless we're holding a piece
      transitionIntake(Input.STOP);
    }
    transitionArm(Input.COMPLETED);
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
      }
    } else if(!isInAuto && queuedCommand != null) {
      // we are no longer in auto, this command no longer applies, clear the queued command
      queuedCommand = null;
    }

    if(wristMovementStartedTime != 0 && Timer.getFPGATimestamp() - wristMovementStartedTime > 0.75) {
      wristMovementStartedTime = 0;
      transitionArm(Input.COMPLETED);
    }


    /*
     * Logic for handling special cases for autonomous where we won't receive a button release event
     */
    if(isInAuto) {
      if(movementType == MovementType.PICKUP && currentIntakeState == IntakeState.HOLDING && 
         (currentArmState == ArmState.EXTENDED || currentArmState == ArmState.WRIST_ONLY_FLEXED)) {
        transitionArm(Input.RETRACT);
      } else if(movementType == MovementType.SCORE && currentArmState == ArmState.EXTENDED) {
        transitionIntake(Input.RELEASE);
        transitionArm(Input.RETRACT);
      }
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

    if(wristMovementStartedTime != 0 && Timer.getFPGATimestamp() - wristMovementStartedTime > 0.75) {
      wristMovementStartedTime = 0;
      transitionArm(Input.COMPLETED);
    }

    
    /*
     * Logic for handling intake cases
     */
    if(currentIntakeState == IntakeState.STARTING && subsystem.isIntakeAtStartedVelocity()) {
      transitionIntake(Input.STARTED);
    }

    if(currentIntakeState == IntakeState.RETRIEVING && subsystem.isIntakeAtHoldingVelocity()) {
      if(movementType == MovementType.PICKUP_DOWNED_CONE) {
        transitionArm(Input.RETRACT);
      } else {
        transitionIntake(Input.RETRIEVED);
      }
    }


    /*
     * Allow for joystick adjustment of the distal arm when extended
     */
    if(currentArmState == ArmState.EXTENDED && distalJoystickControl != null) {
      if(distalJoystickControl.startPosition == 0) {
        System.out.println("ArmStateMachine: Enabling arm/hand position adjustment!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        // set the position only once, we want to stay within reasonable range of the final extended position
        if(distalJoystickControl.adjustWrist) {
          distalJoystickControl.setStartPosition(subsystem.getWristPosition());
        } else {
          distalJoystickControl.setStartPosition(subsystem.getDistalArmPosition());
        }
      }

      if(distalJoystickControl.adjustWrist){
        subsystem.moveWrist(distalJoystickControl.getPositionAdjustment(), ArmConstants.wristMaxVel);
      } else {
        subsystem.adjustDistalArm(distalJoystickControl.getPositionAdjustment());
      }
    }

    /*
     * Handling emergency status including joystick control of the proximal and distal arms
     */
    if(currentArmState == ArmState.EMERGENCY_RECOVERY) {
      // set the position continuously, we want to allow the operator to move the arm as much as they need to
      if(proximalJoystickControl != null) {
        subsystem.adjustProximalArmVelocity(proximalJoystickControl.getVelocityAdjustment());
      }
      if(distalJoystickControl != null) {
        subsystem.adjustDistalArmVelocity(distalJoystickControl.getVelocityAdjustment());
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
      case WRIST_ONLY_FLEXING:
        subsystem.moveWrist(currentWristFlexPosition, ArmStateConstants.wristOnlyFlexMaxVelocity);
        wristMovementStartedTime = Timer.getFPGATimestamp();
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
      case RESETTING_WRIST:
        subsystem.moveWristHome();
        wristMovementStartedTime = Timer.getFPGATimestamp();
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
   * GETTERS/SETTERS, AND HELPER METHODS FOR OTHER SYSTEMS
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

  public HighPickup getHighPickup() {
    return highPickup;
  }

  public void setHighPickup(HighPickup pickup) {
    highPickup = pickup;
  }

  public boolean isHoldingGamePiece() {
    return (currentIntakeState == IntakeState.HOLDING);
  }

  public boolean isInEmergencyRecovery() {
    return (status == Status.EMERGENCY_RECOVERY);
  }

  public MovementType getMovementType() {
    return movementType;
  }

  public void setIsInAuto(boolean inAuto) {
    System.out.println("ArmStateMachine: Setting utonomous mode " + inAuto + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    isInAuto = inAuto;
  }

  public void setAllowScore(boolean allow) {
    allowScore = allow;
  }

  // NOTE: the keypad is not currently in use, we are using a switch on the flight controller instead,
  // however, leaving this code in place in case we switch back
  public void setOperatorSequence(String keypadEntry) {
    if(isRunningOperatorEntry) return; // do not allow keypad change if running an operator sequence

    String[] keypadValues = keypadEntry.split("; ");
    if(keypadValues.length > 1) {
      String sequenceCode = keypadValues[1];
      ArmSequence sequence = ArmSequence.valueForCode(sequenceCode);
      if(sequence != null) {
        operatorSequence = sequence;
      }
    }
  }

  public void setOperatorSequence(int switchId) {
    if(isRunningOperatorEntry) return; // do not allow switch change if running an operator sequence

    ArmSequence sequence = ArmSequence.valueForSwitch(switchId);
    if(sequence != null) {
      operatorSequence = sequence;
    }
  }

  public ArmSequence getOperatorSequence() {
    return operatorSequence;
  }

  public void addJoystickControl(Joystick joystick, int axis, boolean adjustWrist) {
    if(axis == OperatorConsoleConstants.kProximalAxisId) {
      proximalJoystickControl = new JoystickControl(joystick, axis, adjustWrist);
    } else {
      distalJoystickControl = new JoystickControl(joystick, axis, adjustWrist);
    }
  }
}