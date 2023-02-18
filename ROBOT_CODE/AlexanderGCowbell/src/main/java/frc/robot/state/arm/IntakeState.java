package frc.robot.state.arm;

public enum IntakeState {
    STOPPED {
        public IntakeState next(ArmInput input) {
            ArmInput ai = (ArmInput)input;
            switch(ai) {
                case START:
                    return RETRIEVING;
                default:
                    return this;
            }
        }
    },

    RETRIEVING {
        public IntakeState next(ArmInput input) {
            ArmInput ai = (ArmInput)input;
            switch(ai) {
                case RETRIEVED:
                    return HOLDING;
                case STOP:
                    return STOPPED;
                default:
                    return this;
            }
        }
    },

    HOLDING {
        public IntakeState next(ArmInput input) {
            ArmInput ai = (ArmInput)input;
            switch(ai) {
                case RELEASE:
                    return RELEASING;
                default:
                    return this;
            }
        }
    },

    RELEASING {
        public IntakeState next(ArmInput input) {
            ArmInput ai = (ArmInput)input;
            switch(ai) {
                case RELEASED:
                    return STOPPED;
                default:
                    return this;
            }
        }
    };
    
    public IntakeState next(ArmInput input) {
        return this;
      }

}