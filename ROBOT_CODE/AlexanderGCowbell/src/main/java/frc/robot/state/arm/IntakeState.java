package frc.robot.state.arm;

import frc.robot.state.arm.ArmStateMachine.Input;

public enum IntakeState {
    STOPPED {
        public IntakeState next(Input input) {
            switch(input) {
                case START:
                    return STARTING;
                case RETRIEVED:
                    // used in autonomous where we start up in holding
                    return HOLDING; 
                case RELEASE:
                    return RELEASING;
                default:
                    return this;
            }
        }
    },

    STARTING {
        public IntakeState next(Input input) {
            switch(input) {
                case STARTED:
                    return RETRIEVING;
                case STOP:
                    return STOPPED;
                default:
                    return this;
            }
        }
    },

    RETRIEVING {
        public IntakeState next(Input input) {
            switch(input) {
                case DETECT_PIECE:
                    return DETECTING_PIECE;
                case RETRIEVED:
                    return HOLDING;
                case STOP:
                    return STOPPED;
                default:
                    return this;
            }
        }
    },

    DETECTING_PIECE {
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
                case STOP:
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