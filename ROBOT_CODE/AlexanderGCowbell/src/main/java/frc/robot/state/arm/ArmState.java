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
          return WRIST_ONLY_FLEXED;
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

  WRIST_ONLY_FLEXED {
    public ArmState next(Input input) {
      switch(input) {
        case EXTEND_WRIST: // extends (moves home) wrist only w/o moving arm
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
  };

  /* 
  Temporarily disabling these states until we revise this process
  RESETTING_WRIST {
    public ArmState next(Input input) {
      switch(input) {
        case COMPLETED:
          return RESETTING_PROXIMAL;
        default:
          return this;
      }
    }
  },

  RESETTING_PROXIMAL {
    public ArmState next(Input input) {
      switch(input) {
        case COMPLETED:
          return RESETTING_DISTAL;
        default:
          return this;
      }
    }
  },

  RESETTING_DISTAL {
    public ArmState next(Input input) {
      switch(input) {
        case COMPLETED:
          return HOME;
        default:
          return this;
      }
    }
  };
  */
  
  public ArmState next(Input input) {
    return this;
  }
}