package frc.robot.state;

import frc.robot.state.StateMachine.Status;

public class StateChange {
    public Status status;
    public State previousState;
    public State newState;
    public StateChangeRequest request;
    public StateChangeResult result;
    public boolean interrupted = false;
    public double interruptedTimestamp = 0.0;

    public StateChange(Status status, State previousState, State newState, StateChangeRequest request) {
        this(status, previousState, newState, request, null);
    }

    public StateChange(Status status, State previousState, State newState, StateChangeRequest request, StateChangeResult result) {
        this.previousState = previousState;
        this.newState = newState;
        this.request = request;
        this.result = result;
    }
}
