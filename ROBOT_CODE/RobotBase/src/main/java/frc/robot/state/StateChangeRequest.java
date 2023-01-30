package frc.robot.state;

import frc.robot.Constants.StateConstants.StateMachineWaitCondition;

public class StateChangeRequest {
  public Input input;
  public Object data;
  public StateMachineWaitCondition waitCondition;

  public StateChangeRequest(Input input) {
    this(input, null, null);
  }

  public StateChangeRequest(Input input, Object data) {
    this(input, data, null);
  }

  public StateChangeRequest(Input input, StateMachineWaitCondition waitCondition) {
    this(input, null, waitCondition);
  }

  public StateChangeRequest(Input input, Object data, StateMachineWaitCondition waitCondition) {
    this.input = input;
    this.data = data;
    this.waitCondition = waitCondition;
  }
}
