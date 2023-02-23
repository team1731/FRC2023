package frc.robot.util;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class PoseEstimatorLog{

    private DoubleLogEntry xLog;
    private DoubleLogEntry yLog;
    private DoubleLogEntry rotLog;

    public PoseEstimatorLog(){
        DataLog poseLog = DataLogManager.getLog();
        xLog = new DoubleLogEntry(poseLog, "pose/x");
        yLog = new DoubleLogEntry(poseLog, "pose/y");
        rotLog = new DoubleLogEntry(poseLog, "pose/rotation");
    }
    public void append(double x, double y, double rotation){
        xLog.append(x);
        yLog.append(y);
        rotLog.append(rotation);
    }
}