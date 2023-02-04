package frc.robot.state;

import frc.robot.Constants.StateConstants.ResultCode;

public class StateChangeResult {
  public ResultCode code;
  public String message;
  public double timestamp;
  public Object data;

  public StateChangeResult(ResultCode code, String message, double timestamp) {
    this(code, message, timestamp, null);
  }

  public StateChangeResult(ResultCode code, String message, double timestamp, Object data) {
    this.code = code;
    this.message = message;
    this.timestamp = timestamp;
    this.data = data;
  }
}
