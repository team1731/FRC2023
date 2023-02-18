package frc.robot.state.arm;

import frc.robot.state.arm.ArmStateMachine.Input;

public enum IntakeState {
    STOPPED {
        public IntakeState next(Input input) {
            switch(input) {
                case START:
                    return RETRIEVING;
                default:
                    return this;
            }
        }
    },

    RETRIEVING {
        public IntakeState next(Input input) {
            switch(input) {
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
        public IntakeState next(Input input) {
            switch(input) {
                case RELEASE:
                    return RELEASING;
                default:
                    return this;
            }
        }
    },

    RELEASING {
        public IntakeState next(Input input) {
            switch(input) {
                case RELEASED:
                    return STOPPED;
                default:
                    return this;
            }
        }
    };
    
    public IntakeState next(Input input) {
        return this;
      }

}