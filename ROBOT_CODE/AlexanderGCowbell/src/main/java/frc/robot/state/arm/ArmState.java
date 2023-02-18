package frc.robot.state.arm;

public enum ArmState {
    
  HOME {
    public ArmState next(ArmInput input) {
      ArmInput ai = (ArmInput)input;
      switch(ai) {
        case EXTEND:
          return EXTENDING;
        default:
          return this;
      }
    }
  },

  EXTENDING {
    public ArmState next(ArmInput input) {
      ArmInput ai = (ArmInput)input;
      switch(ai) {
        case COMPLETED:
          return EXTENDED;
        case STOP:
          return PAUSED;
        case INTERRUPT:
          return RETRACTING;
        default:
          return this;
      }
    }
  },

  PAUSED {
    public ArmState next(ArmInput input) {
      ArmInput ai = (ArmInput)input;
      switch(ai) {
        case EXTEND:
          return EXTENDING;
        case INTERRUPT:
          return RETRACTING;
        default:
          return this;
      }
    }
  },

  EXTENDED {
    public ArmState next(ArmInput input) {
      ArmInput ai = (ArmInput)input;
      switch(ai) {
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
    public ArmState next(ArmInput input) {
      ArmInput ai = (ArmInput)input;
      switch(ai) {
        case COMPLETED:
          return RESETTING_WRIST;
        default:
          return this;
      }
    }
  },

  RESETTING_WRIST {
    public ArmState next(ArmInput input) {
      ArmInput ai = (ArmInput)input;
      switch(ai) {
        case COMPLETED:
          return RESETTING_PROXIMAL;
        default:
          return this;
      }
    }
  },

  RESETTING_PROXIMAL {
    public ArmState next(ArmInput input) {
      ArmInput ai = (ArmInput)input;
      switch(ai) {
        case COMPLETED:
          return RESETTING_DISTAL;
        default:
          return this;
      }
    }
  },

  RESETTING_DISTAL {
    public ArmState next(ArmInput input) {
      ArmInput ai = (ArmInput)input;
      switch(ai) {
        case COMPLETED:
          return HOME;
        default:
          return this;
      }
    }
  };
  
  public ArmState next(ArmInput input) {
    return this;
  }
}