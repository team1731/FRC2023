package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class AutoCheckRemainingTime extends CommandBase {



    public AutoCheckRemainingTime() {

    }

    @Override
	public void initialize() {
       // System.out.println("Match Time Remaining init" +  Timer.getMatchTime());

	}

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
      //  System.out.println("Match Time Remaining finished" +  Timer.getMatchTime());
        return Timer.getMatchTime() > 2.5;
        
    }
}
