package frc.robot.state;

public class StateChange {
    public State previousState;
    public State newState;
    public StateChangeRequest request;
    public StateChangeResult result;

    public StateChange(State previousState, State newState, StateChangeRequest request) {
        this(previousState, newState, request, null);
    }

    public StateChange(State previousState, State newState, StateChangeRequest request, StateChangeResult result) {
        this.previousState = previousState;
        this.newState = newState;
        this.request = request;
        this.result = result;
    }
}
