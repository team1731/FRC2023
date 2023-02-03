package frc.robot.state;

public class StateChangeResult {
  public String code;
  public String message;
  public double timestamp;
  public Object data;

  public StateChangeResult(String code, String message, double timestamp) {
    this(code, message, timestamp, null);
  }

  public StateChangeResult(String code, String message, double timestamp, Object data) {
    this.code = code;
    this.message = message;
    this.timestamp = timestamp;
    this.data = data;
  }
}
