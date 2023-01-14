/*
    Limelight 2+ Specs:

    RES: 320 x 240 pixels
    FOV: 59.6 x 49.7 degrees
*/

package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.vision.LimeTargetInfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * This subsystem stores the last target coordinates and allows for easy control
 * over the LED
 */
public class VisionSubsystem extends ToggleableSubsystem {

	@Override
	protected boolean getEnabled(){
		return true;
	}

	/**
	 * The table that contains all controls and outputs for the Limelight
	 */
	private NetworkTable _limeTable;
	/**
	 * The current vision pipeline the Limelight is set to
	 */
	private NetworkTableEntry _limePipeline;
	/**
	 * How off the X axis (target coordinates) the target is from the crosshair in
	 * degrees
	 */
	private NetworkTableEntry _limeTX;
	/**
	 * How off the Y axis (target coordinates) the target is from the crosshair in
	 * degrees
	 */
	private NetworkTableEntry _limeTY;
	/**
	 * How much of the image the target covers (0%-100%)
	 */
	private NetworkTableEntry _limeArea;
	/**
	 * The horizontal (width) sidelength of the rough bounding box (0-320 pixels)
	 */
	private NetworkTableEntry _limeHoriz;
	/**
	 * The vertical (length) sidelength of the rough bounding box (0-320 pixels)
	 */
	private NetworkTableEntry _limeVert;
	/**
	 * Indicates if the Limelight has a target. 1 for yes, 0 for no.
	 */
	private NetworkTableEntry _limeValidTargets;
	/**
	 * Control for LED. 1 is off, 2 is blink, 3 is on, and 0 is default for pipeline
	 */
	private NetworkTableEntry _limeLED;

	private NetworkTableEntry _stream;

	private NetworkTableEntry[] _limeRawX = new NetworkTableEntry[3];
	private NetworkTableEntry[] _limeRawY = new NetworkTableEntry[3];
	private NetworkTableEntry[] _limeRawArea = new NetworkTableEntry[3];

	/**
	 * The last target that was reported by the Limelight
	 */
	private LimeTargetInfo _lastTarget = LimeTargetInfo.empty;
	/**
	 * Targets collected over {@link VisionConstants#kAverageKeepTime} seconds
	 */
	private List<LimeTargetInfo> _pastTargets = new ArrayList<LimeTargetInfo>();

	/**
	 * Keeps track of how many systems are requesting the LED. Each system should be
	 * turning off the LED when they are done. e.g. VisionRotateCommand turns the
	 * LED on while the command is active and then turns it off when deactivated.
	 */
	private int _ledQueries = 0;

	public VisionSubsystem() {
		if(isDisabled()) return;

		// Set tables for easy getting
		_limeTable = NetworkTableInstance.getDefault().getTable("limelight");
		_limePipeline = _limeTable.getEntry("pipeline");
		_limeTX = _limeTable.getEntry("tx");
		_limeTY = _limeTable.getEntry("ty");
		_limeArea = _limeTable.getEntry("ta");
		_limeHoriz = _limeTable.getEntry("thor");
		_limeVert = _limeTable.getEntry("tvert");
		_limeValidTargets = _limeTable.getEntry("tv");
		_limeLED = _limeTable.getEntry("ledMode");
		_stream = _limeTable.getEntry("stream");
		for (int i = 0; i < 3; i++) {
			_limeRawX[i] = _limeTable.getEntry("tx" + i);
			_limeRawY[i] = _limeTable.getEntry("ty" + i);
			_limeRawArea[i] = _limeTable.getEntry("ta" + i);
		}

		// Keep the light off so we don't blind unfortunate spectators
		disableLED(false);
		_stream.setNumber(2);
	}

	@Override
	public void periodic() {
		if(isDisabled()) return;

		// Report target when one is valid
		if (hasTarget()) {
			double currentTime = Timer.getFPGATimestamp();
			LimeTargetInfo newTarget = new LimeTargetInfo(_limeTX.getDouble(0), _limeTY.getDouble(0), _limeArea.getDouble(0),
															_limeVert.getDouble(0), _limeHoriz.getDouble(0), currentTime);
			if(!newTarget.equals(LimeTargetInfo.empty)){
				_lastTarget = newTarget;
				_pastTargets.removeIf(target -> currentTime - target.getTimeCaptured() > VisionConstants.kAverageKeepTime);
				_pastTargets.add(_lastTarget);
			}
		}

		UpdateSmartDashboard();
	}

	/**
	 * Updates the Vis_HasTarget and Vis_TargetPos SmartDashboard entries
	 */
	private void UpdateSmartDashboard() {
		if(isDisabled()){
			return;
		}

	//	SmartDashboard.putBoolean("Vis_HasTarget", hasTarget());
	//	SmartDashboard.putString("Vis_TargetPos",
	//			hasTarget() ? _lastTarget.getY() + ", " + _lastTarget.getZ() : "N/A");
	}

	/**
	 * Gets the last target reported by the Limelight
	 * 
	 * @return The last target reported by the Limelight
	 */
	public LimeTargetInfo getLastTarget() {
		if(isDisabled()) return LimeTargetInfo.empty;

		return _lastTarget;
	}

	/**
	 * Gets the average position of targets from the past {@link VisionConstants#kAverageKeepTime} seconds
	 * @return A new target with averaged values
	 */
	public LimeTargetInfo getAveragedTarget() {
		if(isDisabled()) return LimeTargetInfo.empty;

		double avgY = 0;
		double avgZ = 0;
		double avgArea = 0;
		double avgHor = 0;
		double avgVert = 0;
		for(LimeTargetInfo target : _pastTargets){
			avgY += target.getY();
			avgZ += target.getZ();
			avgArea += target.getArea();
			avgHor += target.getWidth();
			avgVert += target.getLength();
		}
		avgY /= _pastTargets.size();
		avgZ /= _pastTargets.size();
		avgArea /= _pastTargets.size();
		avgHor /= _pastTargets.size();
		avgVert /= _pastTargets.size();

		return new LimeTargetInfo(avgY, avgZ, avgArea, avgHor, avgVert, _lastTarget.getTimeCaptured());
	}

	/**
	 * Checks if the Limelight has any valid targets
	 * 
	 * @return Whether or not the Limelight has any valid targets
	 */
	public boolean hasTarget() {
		if(isDisabled()) return false;

		return _limeValidTargets.getDouble(0) > 0;
	}

	/**
	 * Turns on the LED
	 */
	public void enableLED() {
		if(isDisabled()) return;
        _stream.setNumber(2);
		_limeLED.setNumber(3);
		_ledQueries++;
	}

	/**
	 * Notes that your system is done with the LED. If all systems are done with the
	 * LED, it is turned off
	 */
	public void disableLED() {
		if(isDisabled()) return;

		disableLED(true);
	}

	/**
	 * Notes that your system is done with the LED. If all systems are done with the
	 * LED, it is turned off
	 * 
	 * @see disableLED()
	 * @param trackQuery Whether or not to actually track the system that queried
	 *                   the disable. Setting to false typically forces the LED to
	 *                   turn off.
	 */
	public void disableLED(boolean trackQuery) {
		_limeLED.setNumber(0);
		if(isDisabled()) return;
        
		if (trackQuery) {
			_ledQueries--;
		}

		if (_ledQueries <= 0 && trackQuery) {
			_limeLED.setNumber(0);
		}
	}
}