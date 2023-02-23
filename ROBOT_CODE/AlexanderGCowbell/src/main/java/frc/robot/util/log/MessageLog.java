package frc.robot.util.log;

public class MessageLog {
  private static Logger logger;

  static class Message {
    public String message;

    public Message(String messageInput){
      this.message = messageInput;
    }
  }
  
  public static void start() {
    if (logger == null){
      logger = LogWriter.getLogger(LogWriter.Log.MESSAGE, Message.class);
    }
  }

  public static void add(String message) {
    logger.add(new Message(message));
  }

  public static Logger getLogger() {
    return logger;
  }

}
