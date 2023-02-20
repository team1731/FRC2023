package frc.robot.util.log;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.LogConstants;


/*
 * NOTE: LogWriter supports logging the following types
 * boolean, int, long, double, float, String
 * 
 * DataLogManager also supports logging arrays of values, but to simplify things and 
 * smooth out differences in the underlying loggers you should convert arrays of values
 * to a String before sending it to the logger
 */
public class LogWriter {
    public enum Log {
        POSE_ESTIMATIONS, ARM_PATH_RECORDING, MESSAGE
    }

    public enum LogMode {
        CSV,        // Writes to CSV file using ReflectingCSVWriter
        DATA_LOG    // Uses wpilibs DataLogManager
    }

    public static boolean isArmRecordingEnabled() {
        return LogConstants.loggers.get(LogWriter.Log.ARM_PATH_RECORDING).booleanValue();
    }

    public static class MockLogger implements Logger {
        public void suspend() {}
	    public void resume() {}
	    public boolean isSuspended() { return false; }
        public void add(Object value) {}
        public void flush() {}
    }

    public static void setupLogging() {
        if(LogConstants.loggingEnabled && LogConstants.logMode == LogMode.DATA_LOG) {
            DataLogManager.logNetworkTables(LogConstants.logNetworkTables);
            DataLogManager.start();
        }
    }

    public static Logger getLogger(Log log, Class typeClass) {
        Boolean logEnabled = LogConstants.loggers.get(log);
        if(!LogConstants.loggingEnabled || logEnabled == null || !logEnabled.booleanValue()) {
            return new MockLogger(); // disabled, ignore logging attempts
        }

        if(LogConstants.logMode == LogMode.CSV) {
            return new ReflectingCSVWriter(typeClass);
        } else if(LogConstants.logMode == LogMode.DATA_LOG) {
            return new DataLogWriter(typeClass);
        } else {
            return new MockLogger(); // ignore logging attempts
        }
    }
}
