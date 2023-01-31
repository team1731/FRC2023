package frc.robot.state.arm;

import frc.robot.Constants.StateConstants;
import frc.robot.state.*;

public class ArmStateMachine extends StateMachine {

  /*
   * Defines valid sequences that can be supplied to the state machine
   * Note: leave test sequences in place, they are used by the ArmStateMachineTest (JUnit)
   */
  public enum ArmSequence implements StateSequence {
    SCORE_TEST, PICKUP_TEST;

    public String getDescription() {
      return "ArmSequence: " + this.toString();
    }
  };

  /*
   * Defines valid inputs for state transitions
   */
  public enum ArmInput implements Input {
    EXTEND, RETRACT, INTAKE, RELEASE, RECOVER, INTERRUPT, SUCCESS, FAILED;

    public String getDescription() {
      return "ArmStateInput: " + this.toString();
    }
  }

  /*
   * Defines valid transitions that can occur within the state machine
   */
  public enum ArmState implements State {
    RETRACTED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case EXTEND:
            return EXTENDING;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RETRACTING {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case SUCCESS:
            return RETRACTED;
          case FAILED:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    EXTENDING {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case SUCCESS:
            return EXTENDED;
          case FAILED:
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    EXTENDED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case INTAKE:
            return RETRIEVING;
          case RELEASE:
            return RELEASING;
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RETRIEVING {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case SUCCESS:
            return RETRIEVED;
          case FAILED:
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RETRIEVED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case RETRACT:
            return RETRACTING;
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RELEASING {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case SUCCESS:
            return RELEASED;
          case FAILED:
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RELEASED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case RETRACT:
            return RETRACTING;
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },
  
    INTERRUPTED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case RECOVER:
            return RECOVERING;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RECOVERING {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case SUCCESS:
            return RETRACTED;
          case FAILED:
            return UNSAFE;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    UNSAFE {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        handleInvalidTransition(this, input);
        return this;
      }
    };

    public String getDescription() {
      return "ArmState: " + this.toString();
    }

    private static void handleInvalidTransition(State state, Input input) throws StateMachineInvalidTransitionException {
      String message = "Invalid arm state: no transition for " + state + " with input " + input;
      System.out.println(message);
      throw new StateMachineInvalidTransitionException(StateConstants.kArmStateMachineId, message);
    }
  }

  /*
   * Overrides to define the behavior of the Arm State Machine
   */

  public ArmStateMachine(String id, StateHandler stateHandler) {
    super(id, stateHandler);
    state = ArmState.RETRACTED;
  }

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
  protected StateChangeRequest[] defineSequence(StateSequence selectedSequence) throws StateMachineInitializationException {
    ArmSequence ss = (ArmSequence)selectedSequence;
    switch(ss) {
      case SCORE_TEST:
        return StateConstants.kTestSequenceScore;
      case PICKUP_TEST:
        return StateConstants.kTestSequencePickup;
      default:
        throw new StateMachineInitializationException(getId(), "Sequence supplied is not supported " + selectedSequence);
    }
  }

  public void interruptSequence() {
    if(status != Status.READY || status != Status.WAITING) {
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
      transitionState(ArmInput.INTERRUPT);
      if(status == Status.INVALID) {
        return; // something went wrong w/transition
      }
      recover();
    }
  }

  protected void handleFailureCondition(StateChangeResult result) {
    recover();
  }

  protected void handleInvalidTransition() {
    state = ArmState.UNSAFE;
  }

  private void recover() {
    // TODO implement recovery logic once we know what this looks like
    // will need to define a new sequence with calculated positions to retract the arm
  }
}
