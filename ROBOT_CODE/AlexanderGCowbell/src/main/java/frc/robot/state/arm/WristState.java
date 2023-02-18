package frc.robot.state.arm;

import frc.robot.state.arm.ArmStateMachine.Input;

public enum WristState {
    HOME {
        public WristState next(Input input) {
            switch(input) {
                case FLEX:
                    return FLEXED;
                default:
                    return this;
                }
            }
        },
      
    FLEXED {
        public WristState next(Input input) {
            switch(input) {
                case RESET:
                    return HOME;
                default:
                    return this;
            }
        }
    };

    public WristState next(Input input) {
        return this;
    }
}