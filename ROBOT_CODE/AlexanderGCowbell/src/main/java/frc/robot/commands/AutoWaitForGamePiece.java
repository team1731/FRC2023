package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.state.arm.ArmStateMachine;


public class AutoWaitForGamePiece extends CommandBase {
    private ArmStateMachine stateMachine;
    private boolean isFinished = false;


    public  AutoWaitForGamePiece(ArmStateMachine stateMachine) {
        this.stateMachine = stateMachine;

    }

    @Override
	public void initialize() {
        isFinished = false;
	}

    @Override
    public void execute() {
        if(Timer.getMatchTime() <= 0.1) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        if(isFinished) {
            return true;
        } else {
            return stateMachine.isHoldingGamePiece();
        }
    }
}
