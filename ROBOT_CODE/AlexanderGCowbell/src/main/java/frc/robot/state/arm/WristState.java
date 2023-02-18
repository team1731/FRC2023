package frc.robot.state.arm;

public enum WristState {
    HOME {
        public WristState next(ArmInput input) {
            ArmInput ai = (ArmInput)input;
            switch(ai) {
                case FLEX:
                    return FLEXED;
                default:
                    return this;
                }
            }
        },
      
    FLEXED {
        public WristState next(ArmInput input) {
            ArmInput ai = (ArmInput)input;
            switch(ai) {
                case RESET:
                    return HOME;
                default:
                    return this;
            }
        }
    };

    public WristState next(ArmInput input) {
        return this;
      }
}