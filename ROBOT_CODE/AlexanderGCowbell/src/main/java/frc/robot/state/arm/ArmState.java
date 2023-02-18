package frc.robot.state.arm;

import frc.robot.state.arm.ArmStateMachine.Input;

public enum ArmState {
    
  HOME {
    public ArmState next(Input input) {
      switch(input) {
        case EXTEND:
          return EXTENDING;
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

  RETRACTING {
    public ArmState next(Input input) {
      switch(input) {
        case COMPLETED:
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
  
  public ArmState next(Input input) {
    return this;
  }
}