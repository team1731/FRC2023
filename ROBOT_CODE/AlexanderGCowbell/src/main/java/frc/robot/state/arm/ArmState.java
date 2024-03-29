package frc.robot.state.arm;

import frc.robot.state.arm.ArmStateMachine.Input;

public enum ArmState {

  UNKNOWN {
    public ArmState next(Input input) {
      switch(input) {
        case INITIALIZE:
          return RESETTING;
        default:
          return this;
      }
    }
  },
    
  HOME {
    public ArmState next(Input input) {
      switch(input) {
        case EXTEND:
          return EXTENDING;
        case FLEX_WRIST: // flexes the wrist only, w/o moving arm
          return WRIST_ONLY_FLEXING;
        default:
          return this;
      }
    }
  },

  EXTENDING {
    public ArmState next(Input input) {
      switch(input) {
        case COMPLETED:
          return EXTENDED;
        case INTERRUPT:
          return RETRACTING;
        default:
          return this;
      }
    }
  },

  EXTENDED {
    public ArmState next(Input input) {
      switch(input) {
        case RETRACT:
          return RETRACTING;
        case INTERRUPT:
          return RETRACTING;
        default:
          return this;
      }
    }
  },

  WRIST_ONLY_FLEXING {
    public ArmState next(Input input) {
      switch(input) {
        case COMPLETED:
          return WRIST_ONLY_FLEXED;
        case INTERRUPT:
          return RESETTING;
        default:
          return this;
      }
    }
  },

  WRIST_ONLY_FLEXED {
    public ArmState next(Input input) {
      switch(input) {
        case RETRACT: // extends (moves home) wrist only w/o moving arm
        case INTERRUPT:
          return RESETTING;
        default:
          return this;
      }
    }
  },

  RETRACTING {
    public ArmState next(Input input) {
      switch(input) {
        case COMPLETED:
          return RESETTING;
        default:
          return this;
      }
    }
  },

  RESETTING {
    public ArmState next(Input input) {
      switch(input) {
        case COMPLETED:
          return HOME;
        default:
          return this;
      }
    }
  },

  EMERGENCY_RECOVERY {
    public ArmState next(Input input) {
      switch(input) {
        case AUTO_RECOVER:
          return RESETTING_WRIST;
        default:
          return this;
      }
    }
  },

  RESETTING_WRIST {
    public ArmState next(Input input) {
      switch(input) {
        case COMPLETED:
          return RESETTING;
        default:
          return this;
      }
    }
  };
  
  public ArmState next(Input input) {
    return this;
  }
}