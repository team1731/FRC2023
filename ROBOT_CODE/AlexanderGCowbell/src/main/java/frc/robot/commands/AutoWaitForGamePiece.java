package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.state.arm.ArmStateMachine;


public class AutoWaitForGamePiece extends CommandBase {
    private ArmStateMachine stateMachine;


    public  AutoWaitForGamePiece(ArmStateMachine stateMachine) {
        this.stateMachine = stateMachine;

    }

    @Override
	public void initialize() {

	}

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return stateMachine.isHoldingGamePiece();
    }
}
