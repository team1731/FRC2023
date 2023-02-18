package frc.robot.state.arm;

public enum ArmSequence {
    SCORE_HIGH("SCORE_HIGH"),
    SCORE_MEDIUM("SCORE_MEDIUM"),
    SCORE_LOW("SCORE_LOW"),
    PICKUP_HIGH("PICKUP_HIGH"),
    PICKUP_LOW("PICKUP_LOW"),

    // Used for unit tests
    SCORE_TEST("ABC"), 
    PICKUP_TEST("XYZ"), 
    UNDEFINED_TEST(null), 
    INVALID_TEST(null);

    public final String code;

    private ArmSequence(String code) {
        this.code = code;
    }

    public static ArmSequence valueForCode(String code) {
        for (ArmSequence as : values()) {
            if (as.code.equals(code)) {
                return as;
            }
        }
        return null;
    }
};
