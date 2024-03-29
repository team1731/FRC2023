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
  private boolean isInThiefMode = false;
  private boolean isWaitingForIntakeToSlow = false;
  private boolean isRunningOperatorEntry = false;
  private ArmSequence operatorSequence = ArmSequence.SCORE_HIGH; // sequence pre-loaded by operator via keypad/switch

  private ArmState currentArmState = ArmState.UNKNOWN;
  private IntakeState currentIntakeState = IntakeState.STOPPED;
  private boolean allowScore = true;
  private boolean emergencyModeTriggeredNotConfirmed = false;
  private boolean processAutoRecoveryOnRetraction = false;
  private boolean autoDelayScore = false; // should be set to true when autonmous path should add delay before scoring
  private double autoDelayStartTime = 0;

  private ArmPath currentPath; // used if running full arm path
  private double currentWristFlexPosition = 0; // used if running wrist only movement
  private double currentPathQueuedTime; // used to distinguish requests when one running and another queued
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
    public boolean autoCommand;

    public QueuedCommand(MovementType type, ArmPath path, double queuedTime) {
      this.type = type;
      this.path = path;
      this.queuedTime = queuedTime;
      this.autoCommand = isInAuto;
    }

    public QueuedCommand(MovementType type, double wristFlexPosition, double queuedTime) {
      this.type = type;
      this.wristFlexPosition = wristFlexPosition;
      this.queuedTime = queuedTime;
      this.autoCommand = isInAuto;
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
        return startPosition + (joystick.getRawAxis(axis) * ArmConstants.distalMaxAdjustmentTicks * -1);
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
    System.out.println("ArmStateMachine: Resetting State!!!!!!!!!!!!!!!!!!!!!");
    status = Status.READY;
    currentPath = null;
    currentWristFlexPosition = 0;
    currentPathQueuedTime = 0;
    pathStartedIndex = 0;
    pathStartedTime = 0;
    wristMovementStartedTime = 0;
    autoDelayScore = false;
    autoDelayStartTime = 0;
    movementType = null;
    proximalJoystickControl = null;
    distalJoystickControl = null;
    wristFlexed = false;
    allowScore = true;
    emergencyModeTriggeredNotConfirmed = false;
    processAutoRecoveryOnRetraction = false;
    isWaitingForIntakeToSlow = false;

    if(isRunningOperatorEntry) {
      isRunningOperatorEntry = false;
    }
  }

  // put arm into unknown state whenever disabled
  public void disable() {
    System.out.println("ArmStateMachine: DISABLED!!!!!!!!!!!!!!!!!!!!!!!!!");
    resetState();
    currentArmState = ArmState.UNKNOWN;
    queuedCommand = null; // clear out any queued commands from previous mode (e.g., autonomous)
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
  public void pickup(ArmPath path, double queuedTime) {
    pickup(path, MovementType.PICKUP, queuedTime);
  }

  public void pickup(ArmPath path, MovementType movement, double queuedTime) {
    if(path == null) return;
    if(!isReadyToStartMovement()) {
      setQueuedCommand(movement, path, queuedTime);
      return;
    }

    System.out.println("ArmStateMachine: STARTING " + movement + "!!!!!!!!!!!!!!!!!!!!!");
    currentPath = path;
    currentPathQueuedTime = queuedTime;
    pathStartedIndex = 0;
    movementType = movement;
    // make sure the intake is stopped before attempting to start it
    transitionIntake(Input.STOP); 
    startPickupIntake();
    // start the arm path
    transitionArm(Input.EXTEND);
  }

  // PICKUP WITH A WRIST FLEX ONLY
  public void pickup(double wristFlexPosition, double queuedTime) {
    if(!isReadyToStartMovement()) {
      setQueuedCommand(MovementType.PICKUP, wristFlexPosition, queuedTime);
      return;
    }

    System.out.println("ArmStateMachine: STARTING WRIST ONLY PICKUP!!!!!!!!!!!!!!!!!!!!!");
    currentWristFlexPosition = wristFlexPosition;
    currentPathQueuedTime = queuedTime;
    movementType = MovementType.PICKUP;
    // make sure the intake is stopped before attempting to start it
    transitionIntake(Input.STOP);
    startPickupIntake();
    // move the wrist into position
    transitionArm(Input.FLEX_WRIST);
    status = Status.RUNNING;
  }

  private void startPickupIntake() {
    if(isInAuto && !subsystem.isIntakeBelowStartedVelocity()) {
      isWaitingForIntakeToSlow = true;
      System.out.println("ArmStateMachine: want to start, but intake hasn't slowed down enough. Waiting for intake to slow.");
    } else {
      transitionIntake(Input.START);
    }
  }

  // SCORE
  public void score(ArmPath path, double queuedTime) {
    if(path == null) return;
    if(!isReadyToStartMovement()) {
      setQueuedCommand(MovementType.SCORE, path, queuedTime);
      return;
    } 

    System.out.println("ArmStateMachine: STARTING SCORE!!!!!!!!!!!!!!!!!!!!!");
    currentPath = path;
    currentPathQueuedTime = queuedTime;
    pathStartedIndex = 0;
    movementType = MovementType.SCORE;
    transitionArm(Input.EXTEND);

    // For cone scores in auto, add a delay to allow arm to stop bouncing before scoring
    if(isInAuto && gamePiece == GamePiece.CONE) {
      autoDelayScore = true;
    }
  }

  // SCORE BASED ON OPERATOR ENTRY (KEYPAD OR SWITCH)
  public void scoreOperatorEntry(double queuedTime) {
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
      score(path, queuedTime);
    }
  }

  // NOTIFY THAT CALLING COMMAND IS ENDING
  // e.g., driver released button, auto command closing out
  // should kick off a retraction
  public void handleCommandEnding(double queuedTime) {
    if(queuedTime != currentPathQueuedTime) {
      if(queuedCommand != null && queuedTime == queuedCommand.queuedTime) {
        // if button is released before the queued command has been run then clear it out
        System.out.println("ArmStateMachine: command canceled, clearing from the queue, queued time: " + queuedTime);
        queuedCommand = null;
      }
      return;
    }

    System.out.println("ArmStateMachine: command ending/interrupted, queued time: " + queuedTime);

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
      // if we detect this condition, we will kick off auto-recovery when we go into retracting
      checkForAccidentalButtonPress(); 
      // start interrupt
      interrupt();
    }
  }

  private boolean isReadyToStartMovement() {
    if(status != Status.READY ||
       currentArmState != ArmState.HOME ||
       currentPath != null) {
      /* Commenting out for the moment to prevent logging during periodic
      System.out.println("WARNING: state machine failed on readiness check --> Status: " + status + 
        ", ArmState: " + currentArmState + 
        ", Path Already Loaded? " + (currentPath != null)
      );*/
      return false;
    }
    return true;
  }

  private void initiateRetraction() {
    System.out.println("ArmStateMachine: retraction initiated by a command ending or an interruption");
    if(currentArmState == ArmState.EXTENDED) {
      transitionArm(Input.RETRACT);
    } else if(isInInterruptibleArmState()) { // if NOT interruptible ignore this request
      // it IS interruptible, likely the arm is still in the middle of an extension
      // ensure the arm is stopped, then process interruption
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
    subsystem.intake(ArmConstants.onDemandIntakeSpeed);
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
    // clear current path is used to handle situation in which we have hit an 
    // edge case where the state machine and subsystem are out of sync, but 
    // the arm is in a safe state for auto recovery
    // allows driver to kick this off without waiting for operator
    status = Status.EMERGENCY_RECOVERY;
    currentArmState = ArmState.EMERGENCY_RECOVERY;
    queuedCommand = null;
    attemptAutoRecovery();
  }

  public int getPathIndex() {
    if(currentPath == null) return 0;

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

  // DETECT IF A RELEASE EVENT WAS LIKELY CAUSED BY AN ACCIDENTAL BUTTON CLICK
  // return = true, means detected accidental click and handled interrupt
  // return = false, means did not detect accidental click and did nothing
  public void checkForAccidentalButtonPress() {
    if(Timer.getFPGATimestamp() - currentPathQueuedTime < 0.5) {
      System.out.println("ArmStateMachine: Detected likely accidental button click!!!!!!!!!!!!!!!!!!!!");
      processAutoRecoveryOnRetraction = true; // as soon as the interrupt takes us into retraction attempt auto-recovery
    } else if(queuedCommand != null && Timer.getFPGATimestamp() - queuedCommand.queuedTime < 0.5) {
      System.out.println("ArmStateMachine: Detected likely accidental button click!!!!!!!!!!!!!!!!!!!!");
      queuedCommand = null; // clear the queued command
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
    if(currentPath == null && status == Status.RUNNING) {
      // this should never be the case, if it is then log as much as we can
      System.err.println("ArmStateMachine: Current path object became null during the path run!!!!!!!!!!!!!!!!!!!!!!!!");
      logDebugInfo();
    }

    /*
     * Logic for handling queued commands
     * These commands can get queued when the command is requested while the arm is still reaching a ready state
     */
    if(!isInAuto && queuedCommand != null && queuedCommand.autoCommand) {
      // we are no longer in auto, this command no longer applies, clear the queued command
      System.out.println("ArmStateMachine: auto command queued, but we are no longer in autonomous, clearing out, queued time: " + queuedCommand.queuedTime);
      queuedCommand = null;
    } else if(queuedCommand != null && isReadyToStartMovement()) {
      System.out.println("ArmStateMachine: we have a queued command and we are ready to start movement");
      if((queuedCommand.type == MovementType.PICKUP || queuedCommand.type == MovementType.PICKUP_DOWNED_CONE) && queuedCommand.path != null) {
        System.out.println("ArmStateMachine: loading queued arm pickup command, queued time: " + queuedCommand.queuedTime);
        pickup(queuedCommand.path, queuedCommand.type, queuedCommand.queuedTime);
        queuedCommand = null;
      } else if(queuedCommand.type == MovementType.PICKUP && queuedCommand.path == null) {
        System.out.println("ArmStateMachine: loading queued wrist pickup command, queued time: " + queuedCommand.queuedTime);
        pickup(queuedCommand.wristFlexPosition, queuedCommand.queuedTime);
        queuedCommand = null;
      } else if(queuedCommand.type == MovementType.SCORE) {
        System.out.println("ArmStateMachine: loading queued score command, queued time: " + queuedCommand.queuedTime);
        score(queuedCommand.path, queuedCommand.queuedTime);
        queuedCommand = null;
      } else {
        System.out.println("ArmStateMachine: we have a queued command, but it didn't match any of the normal criteria. Movement: " + 
          queuedCommand.type + ", path null? " + (queuedCommand.path == null) + ", queued time: " + queuedCommand.queuedTime);
      }
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
        System.out.println("ArmStateMachine: kicking off retraction - we are in auto, picking up, arm extended, and the intake is in a holding state");
        transitionArm(Input.RETRACT);
      } else if(movementType == MovementType.SCORE && currentArmState == ArmState.EXTENDED) {
        if(!autoDelayScore || (autoDelayStartTime != 0 && Timer.getFPGATimestamp() - autoDelayStartTime >= ArmStateConstants.autoScoreConeDelay)) {
          transitionIntake(Input.RELEASE);
          transitionArm(Input.RETRACT);
        }
      }
    }


    /*
     * Logic for flexing/extending wrist
     */
    if(currentPath != null && !wristFlexed && subsystem.isMotionProfileRunning() && subsystem.getDirection() == Direction.FORWARD) {
      int currentIndex = getPathIndex();
      int wristFlexIndex = currentPath.getWristFlexIndex();
      if(currentIndex >= wristFlexIndex) {
        // move the wrist into the flexed position for this path
        subsystem.moveWrist(currentPath.getWristFlexPosition(), currentPath.getWristMaxVelocity());
        wristFlexed = true;
      }
    } else if(currentPath != null && wristFlexed && subsystem.isMotionProfileRunning() && subsystem.getDirection() == Direction.REVERSE) {
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
    if(isWaitingForIntakeToSlow && subsystem.isIntakeBelowStartedVelocity()) {
      isWaitingForIntakeToSlow = false;
      transitionIntake(Input.START);
      System.out.println("ArmStateMachine: Intake has slowed down enough, kicked off intake start for pickup.");
    }

    if(currentIntakeState == IntakeState.STARTING && subsystem.isIntakeAtStartedVelocity()) {
      transitionIntake(Input.STARTED);
    }

    if(currentIntakeState == IntakeState.RETRIEVING && subsystem.isIntakeAtHoldingVelocity()) {
      if(movementType == MovementType.PICKUP_DOWNED_CONE) {
        transitionArm(Input.RETRACT);
      } else {
        System.out.println("ArmStateMachine: moving intake to holding state, it looks like we have retrieved a piece.");
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

    // this condition occurs when we go into an interrupt state immediately after starting a path 
    // due to an accidental button click, this puts us into an out of sync state w/the subsystem
    // as soon as we detect that the interruption has occurred, kick off path clearing + auto-recovery
    if(processAutoRecoveryOnRetraction && currentArmState == ArmState.RETRACTING) {
      clearCurrentPath(); // kick off auto recovery
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
        if(autoDelayScore) {
          autoDelayStartTime = Timer.getFPGATimestamp();
        }
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
      case WRIST_ONLY_FLEXED:
      case EMERGENCY_RECOVERY:
        // these cases represent valid transition states that don't have a corresponding subsystem call
        // putting cases here so that we will not print unnecessary warning messages for them
        break;
      default:
        System.out.println("WARNING: Invalid arm input sent to state machine: " + input + " --> " + newState);
    }
  }

  private boolean isInInterruptibleArmState() {
    ArmState prevState = currentArmState;
    ArmState newState = currentArmState.next(Input.INTERRUPT);
    boolean isInterruptible = (prevState != newState); // if they are equal the state transition is invalid
    System.out.println("ArmStateMachine: interruptible condition check. The current arm state " + currentArmState + " " + (isInterruptible? "is" : "is not") + " interruptible");
    return isInterruptible;
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

  public double getCurrentPathQueuedTime() {
    return currentPathQueuedTime;
  }

  public void setIsInAuto(boolean inAuto) {
    System.out.println("ArmStateMachine: Setting autonomous mode " + inAuto + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    isInAuto = inAuto;
  }

  public boolean isInThiefMode() {
    return isInThiefMode;
  }

  public void setIsInThiefMode(boolean inThiefMode) {
    System.out.println("ArmStateMachine: Setting thief mode " + inThiefMode + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    isInThiefMode = inThiefMode;
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

  private void setQueuedCommand(MovementType movementType, ArmPath path, double queuedTime) {
    if(isInAuto || isReturningToHome()) { // only allow queuing when: auto = anytime, teleop = when returning home
      // allow current queued command to be overwritten
      System.out.println("ArmStateMachine: writing arm command to the queue, movement: " + movementType + " queued time: " + queuedTime);
      queuedCommand = new QueuedCommand(movementType, path, queuedTime);
    }
  }

  private void setQueuedCommand(MovementType movementType, double position, double queuedTime) {
    if(isInAuto || isReturningToHome()) { // only allow queuing when: auto = anytime, teleop = when returning home
      // allow current queued command to be overwritten
      System.out.println("ArmStateMachine: writing wrist command to the queue, movement: " + movementType + " queued time: " + queuedTime);
      queuedCommand = new QueuedCommand(movementType, position, queuedTime);
    }
  }

  private boolean isReturningToHome() {
    if(currentArmState == ArmState.RETRACTING || currentArmState == ArmState.RESETTING || currentArmState == ArmState.RESETTING_WRIST) {
      return true;
    }
    return false;
  }

  /*
   * Debug logging
   */
  private void logDebugInfo() {
    try {
      System.out.println("ArmStateMachine: Starting debug!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      System.out.println("Status: " + status);
      System.out.println("In Auto? " + isInAuto);
      System.out.println("Arm State: " + currentArmState);
      System.out.println("Intake State: " + currentIntakeState);
      System.out.println("Game Piece: " + gamePiece);
      System.out.println("Movement Type: " + movementType);
      if(currentPath != null) {
        System.out.println("Current Path? Yes, # Points:" + currentPath.getNumberOfPoints());
      } else {
        System.out.println("Current Path? No");
      }
      System.out.println("Subsystem running a MP? " + subsystem.isMotionProfileRunning());
      System.out.println("Wrist Flex Position: " + currentWristFlexPosition);
      System.out.println("Path Started Time: " + pathStartedTime);
      System.out.println("Wrist Movement Started Time: " + wristMovementStartedTime);
      System.out.println("Wrist Flexed? " + wristFlexed);
      System.out.println("Allowing Score? " + allowScore);
      System.out.println("High Pickup Mode: " + highPickup);
      System.out.println("Running Operator Entry? " + isRunningOperatorEntry);
      if(isRunningOperatorEntry) {
        System.out.println("Operator Sequence: " + operatorSequence);
      }
    } catch(Exception e) {
      // fail silently
      // if we accidentally put something dumb in here, we never want this to throw an exception
    }
  }
}