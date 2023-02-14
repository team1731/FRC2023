package frc.robot.util.log;

public interface Logger {
    public void suspend();
	public void resume();
	public boolean isSuspended();
    public void add(Object value);
    public void flush();
}
