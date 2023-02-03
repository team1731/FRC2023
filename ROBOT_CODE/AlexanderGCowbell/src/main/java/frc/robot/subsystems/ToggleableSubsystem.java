package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Deprecated
public abstract class ToggleableSubsystem extends SubsystemBase {

	protected abstract boolean getEnabled();

	public boolean isEnabled() {
		return getEnabled();
	};

	public boolean isDisabled() {
		return !getEnabled();
	}

}
