package frc.robot.state.arm;

import frc.robot.state.*;
import frc.robot.Constants.StateConstants;

/*
 * Defines valid transitions that can occur within the state machine
 */
public enum ArmState implements State {
    RETRACTED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case EXTEND_INIT:
            return EXTEND_REQUESTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    EXTEND_REQUESTED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case SUCCESS:
            return EXTEND_INITIALIZED;
          case FAILED:
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    EXTEND_INITIALIZED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case EXTEND_MOVE:
            return EXTEND_MOVE_REQUESTED;
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    EXTEND_MOVE_REQUESTED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case SUCCESS:
            return EXTEND_MOVING;
          case FAILED:
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    EXTEND_MOVING {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case EXTEND_PING:
            return EXTEND_PINGING;
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    EXTEND_PINGING {
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
          case RETRACT_INIT:
            return RETRACT_REQUESTED;
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
          case RETRACT_INIT:
            return RETRACT_REQUESTED;
          case INTERRUPT:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RETRACT_REQUESTED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case SUCCESS:
            return RETRACT_INITIALIZED;
          case FAILED:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RETRACT_INITIALIZED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case RETRACT_MOVE:
            return RETRACT_MOVE_REQUESTED;
          case FAILED:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RETRACT_MOVE_REQUESTED {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case SUCCESS:
            return RETRACT_MOVING;
          case FAILED:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RETRACT_MOVING {
      public State next(Input input) throws StateMachineInvalidTransitionException {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
          case RETRACT_PING:
            return RETRACT_PINGING;
          case FAILED:
            return INTERRUPTED;
          default:
            handleInvalidTransition(this, input);
            return this;
        }
      }
    },

    RETRACT_PINGING {
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
