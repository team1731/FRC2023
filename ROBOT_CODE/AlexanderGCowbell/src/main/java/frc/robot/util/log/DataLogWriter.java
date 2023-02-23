package frc.robot.util.log;

import java.lang.reflect.Field;
import java.util.HashMap;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;


public class DataLogWriter<T> implements Logger {
    private DataLog dataLog;
    private String topic;
    private Field[] fields;
    private HashMap<String, DataLogEntry> loggers;
    private boolean suspended;


    public DataLogWriter(Class<T> typeClass) {
        topic = typeClass.getSimpleName();
		fields = typeClass.getFields();
        loggers = new HashMap<String, DataLogEntry>();

		dataLog = DataLogManager.getLog();
		
		for(Field field : fields) {
            String fieldName = field.getName();
            String topicField = topic + "/" + fieldName;
            Class<T> fieldType = (Class<T>)field.getType();

            DataLogEntry entry = null;
            if(fieldType.isAssignableFrom(Boolean.TYPE)) {
                entry = new BooleanLogEntry(dataLog, topicField);
            } else if(fieldType.isAssignableFrom(Integer.TYPE) || fieldType.isAssignableFrom(Long.TYPE)) {
                entry = new IntegerLogEntry(dataLog, topicField);
            } else if(fieldType.isAssignableFrom(Double.TYPE)) {
                entry = new DoubleLogEntry(dataLog, topicField);
            } else if(fieldType.isAssignableFrom(Float.TYPE)) {
                entry = new FloatLogEntry(dataLog, topicField);
            } else if(fieldType.isAssignableFrom(String.class)) {
                entry = new StringLogEntry(dataLog, topicField);
            }

            loggers.put(fieldName, entry);
		}
    }

	public void suspend() {
		suspended = true;
		flush();
	}

	public void resume() {
		suspended = false;
	}

	public boolean isSuspended() {
		return suspended;
	}

    public void add(Object entry) {
        for(Field field : fields) {
            DataLogEntry entryLogger = null;

            try {
                entryLogger = loggers.get(field.getName());
                if(entryLogger == null) {
                    continue;
                }

                if(entryLogger.getClass().isAssignableFrom(BooleanLogEntry.class)) {
                    boolean fieldValue = (boolean)field.get(entry);
                    ((BooleanLogEntry)entry).append(fieldValue);
                } else if(entryLogger.getClass().isAssignableFrom(IntegerLogEntry.class)) {
                    long fieldValue = (long)field.get(entry);
                    ((IntegerLogEntry)entry).append(fieldValue);
                } else if(entryLogger.getClass().isAssignableFrom(DoubleLogEntry.class)) {
                    double fieldValue = (double)field.get(entry);
                    ((DoubleLogEntry)entry).append(fieldValue);
                } else if(entryLogger.getClass().isAssignableFrom(FloatLogEntry.class)) {
                    float fieldValue = (float)field.get(entry);
                    ((FloatLogEntry)entry).append(fieldValue);
                } else if(entryLogger.getClass().isAssignableFrom(StringLogEntry.class)) {
                    String fieldValue = (String)field.get(entry);
                    ((StringLogEntry)entry).append(fieldValue);
                }
                
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            } catch(IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

    // this method does nothing in DataLogWriter
    // file writing is left to the DataLogManager
    public void flush() {
    }
}
