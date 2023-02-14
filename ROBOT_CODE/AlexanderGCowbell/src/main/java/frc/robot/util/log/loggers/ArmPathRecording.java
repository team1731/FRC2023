package frc.robot.util.log.loggers;

public class ArmPathRecording {
    public final double proximal_pos;
    public final double distal_pos;
    public final double proximal_vel;
    public final double distal_vel;
    public final double wrist_pos;

    public ArmPathRecording(double proximal_pos, double proximal_vel, double distal_pos, double distal_vel, double wrist_pos) {
        this.proximal_pos = proximal_pos;
        this.proximal_vel = proximal_vel;
        this.distal_pos = distal_pos;
        this.distal_vel = distal_vel;
        this.wrist_pos = wrist_pos;
    }
}
