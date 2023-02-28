package frc.robot.state.arm;

/*
 * Note: if the sequence has a keypad mapping the appropriate keypad input should be 
 * set for the 'code' and otherwise the code should be left null
 */

public enum ArmSequence {
    READ_KEYPAD,
    SCORE_HIGH("DEPLOY_HIGH"),
    SCORE_MEDIUM("DEPLOY_MID"),
    SCORE_LOW("DEPLOY_LOW"),
    PICKUP_HIGH,
    PICKUP_LOW,
    PICKUP_LOW_CUBE; // allow this to be kicked off individually, also used to flip a downed cone

    public String code = null;

    private ArmSequence() {
    }

    private ArmSequence(String code) {
        this.code = code;
    }

    public static ArmSequence valueForCode(String code) {
        for(ArmSequence as : values()) {
            if(as.code != null && as.code.equals(code)) {
                return as;
            }
        }
        return null;
    }
};
