package frc.robot.state;

public class StateChangeRequest {
  public Input input;
  public Object data;
  public StateWaitCondition waitCondition;
  public double timestamp;

  public StateChangeRequest(Input input) {
    this(input, null, null);
  }

  public StateChangeRequest(Input input, Object data) {
    this(input, data, null);
  }

  public StateChangeRequest(Input input, StateWaitCondition waitCondition) {
    this(input, null, waitCondition);
  }

  public StateChangeRequest(Input input, Object data, StateWaitCondition waitCondition) {
    this.input = input;
    this.data = data;
    this.waitCondition = waitCondition;
  }
}
