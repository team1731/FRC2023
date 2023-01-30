package frc.robot.state;

public class StateChangeResult {
  public String code;
  public String message;
  public long timestamp;
  public Object data;

  public StateChangeResult(String code, String message, long timestamp) {
    this(code, message, timestamp, null);
  }

  public StateChangeResult(String code, String message, long timestamp, Object data) {
    this.code = code;
    this.message = message;
    this.timestamp = timestamp;
    this.data = data;
  }
}
