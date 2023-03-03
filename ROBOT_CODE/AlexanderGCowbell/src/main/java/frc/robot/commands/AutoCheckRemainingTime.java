package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class AutoCheckRemainingTime extends CommandBase {



    public AutoCheckRemainingTime() {

    }

    @Override
	public void initialize() {

	}

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return Timer.getMatchTime() > 3.0;
    }
}
