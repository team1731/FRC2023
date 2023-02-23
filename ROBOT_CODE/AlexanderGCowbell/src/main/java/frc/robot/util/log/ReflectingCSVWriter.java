package frc.robot.util.log;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.concurrent.ConcurrentLinkedDeque;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Writes data to a CSV file
 */
public class ReflectingCSVWriter<T> implements Logger {
	ConcurrentLinkedDeque<String> mLinesToWrite = new ConcurrentLinkedDeque<>();
	PrintWriter mOutput = null;
	Field[] mFields;

	private boolean suspended;

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

	public ReflectingCSVWriter(Class<T> typeClass) {
		String pathName;
		if (RobotBase.isReal()) {
			pathName = "/home/lvuser/" + typeClass.getSimpleName() + ".csv";
		} else {
			pathName = typeClass.getSimpleName() + ".csv";
		}

		//
		// rename existing file so it's
		// available after we cycle power
		//
		File existingFile = new File(pathName);
		if (existingFile.exists()) {
			File previousFile = new File(pathName + ".prev");
			if (previousFile.exists()) {
				previousFile.delete();
			}
			existingFile.renameTo(previousFile);
		}

		mFields = typeClass.getFields();
		try {
			mOutput = new PrintWriter(pathName);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}

		// Write field names.
		StringBuffer line = new StringBuffer();
		for (Field field : mFields) {
			if (line.length() != 0) {
				line.append(", ");
			}
			line.append(field.getName());
		}
		writeLine(line.toString());
	}
	
	public void add(Object value) {
		if (!suspended) {
			StringBuffer line = new StringBuffer();
			for (Field field : mFields) {
				if (line.length() != 0) {
					line.append(", ");
				}
				try {
					line.append(field.get(value).toString());
				} catch (IllegalArgumentException e) {
					e.printStackTrace();
				} catch (IllegalAccessException e) {
					e.printStackTrace();
				}
			}
			mLinesToWrite.add(line.toString());
		}
	}

	private synchronized void writeLine(String line) {
		if (mOutput != null) {
			mOutput.println(line);
		}
	}

	private void write() {
		while (true) {
			String val = mLinesToWrite.pollFirst();
			if (val == null) {
				break;
			}
			writeLine(val);
		}
	}

	// Call this periodically from any thread to write to disk.
	public synchronized void flush() {
		if (mOutput != null) {
			write();
			mOutput.flush();
		}
	}
}
