package frc.robot.state.arm;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.StateConstants;
import frc.robot.state.*;

public class ArmStateMachine extends StateMachine {
  
  /*
   * Overrides to define the behavior of the Arm State Machine
   */

  public ArmStateMachine(String id, StateHandler stateHandler) {
    super(id, stateHandler);
    state = ArmState.RETRACTED;
  }

  // this method should be used with caution, it can be used to reset the state machine and it 
  // should definitely not be called unless it is certain that the arm is in a safe retracted state
  // there may not be a use case for this method outside of unit testing
  public void reset() {
    state = ArmState.RETRACTED;
  }

  @Override
  protected void initializationChecks() throws StateMachineInitializationException {
    if(state == ArmState.UNSAFE) {
      throw new StateMachineInitializationException(getId(), "Arm is in UNSAFE state. No actions can be performed.");
    } else if(state != ArmState.RETRACTED) {
      // should always be starting from a retracted state
      throw new StateMachineInitializationException(getId(), "Arm is in unexpected state: " + state + ". Should be RETRACTED.");
    }
  }

  /*
   * Retrieves the sequence config matching the provided sequence, and maps them into an iterator of StateChangeRequest objects
   * Note: leave test sequences in place, they are used by the ArmStateMachineTest (JUnit)
   */
  @Override
  protected StateChangeRequest[] defineSequence(StateSequence selectedSequence) throws StateMachineInitializationException {
    ArmSequence ss = (ArmSequence)selectedSequence;
    switch(ss) {
      case SCORE_TEST:
        return StateConstants.kTestSequenceScore;
      case PICKUP_TEST:
        return StateConstants.kTestSequencePickup;
      case INVALID_TEST:
        return StateConstants.kTestInvalid;
      default:
        throw new StateMachineInitializationException(getId(), "Sequence supplied is not supported " + selectedSequence);
    }
  }

  @Override
  public void interruptSequence() {
    if(!isInInterruptibleStatus()) {
      return; // not interruptible
    }

    // check to see if we are in an interruptible state
    boolean shouldInterrupt = false;
    ArmState currentState = (ArmState)state;
    switch(currentState) {
      case RETRACTED: case RETRACTING: case INTERRUPTED: case RECOVERING: case UNSAFE:
        return; // not interruptible
      default:
        shouldInterrupt = true; // if we got here, we are in an interruptible status and state
    }

    if(shouldInterrupt) {
      getStateHandler().interruptStateChange();
      previouState = state;
      transitionState(ArmInput.INTERRUPT);
      if(status == Status.INVALID) {
        return; // something went wrong w/transition
      } else {
        status = Status.INTERRUPTED;
      }

      // Add current step we interrupted to the processed steps list
      var change = new StateChange(previouState, state, currentStep);
      change.interrupted = true;
      change.interruptedTimestamp = Timer.getFPGATimestamp();
      processedSteps.add(change);

      // now initiate recovery
      recover();
    }
  }

  @Override
  protected void handleFailureCondition(StateChangeResult result) {
    recover();
  }

  @Override
  protected void handleInvalidTransition() {
    state = ArmState.UNSAFE;
  }

  private void recover() {
    // calculate a recovery path for the arm
    var data = generateRecoveryData();

    // redefine the state sequence w/recovery step
    var step = new StateChangeRequest(ArmInput.RECOVER, data);
    var sequenceList = new ArrayList<StateChangeRequest>();
    sequenceList.add(step);
    sequence = sequenceList.iterator();

    // advance the iterator to pre-load this step
    currentStep = sequence.next();

    // restart processing to initiate recovery
    processCurrentSequenceStep();
  }

  private Object generateRecoveryData() {
    // TODO, replace this, just passing some dummy data for the moment
    return new double[]{ 11, 12, 13, 14, 15 };
  }
}
