package frc.robot.state;

public class StateWaitCondition {

    public enum WaitType {
        PAUSE, PING
    }

    public Wait condition;
    public WaitType type;

    public StateWaitCondition(Wait condition, WaitType type) {
        this.condition = condition;
        this.type = type;
    }
}
