package frc.robot.state.arm;

public enum ArmSequence {
    READ_KEYPAD(null),
    SCORE_HIGH("SCORE_HIGH"),
    SCORE_MEDIUM("SCORE_MEDIUM"),
    SCORE_LOW("SCORE_LOW"),
    PICKUP_HIGH("PICKUP_HIGH"),
    PICKUP_LOW("PICKUP_LOW"),

    // Used for unit tests
    SCORE_TEST("SCORE_TEST"), 
    PICKUP_TEST("PICKUP_TEST"), 
    UNDEFINED_TEST("UNDEFINED_TEST"), 
    INVALID_TEST("INVALID_TEST");

    public String code;

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
