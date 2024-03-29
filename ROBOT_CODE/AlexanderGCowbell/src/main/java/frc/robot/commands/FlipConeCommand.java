package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.ArmStateConstants;
import frc.robot.state.arm.ArmStateMachine;

public class FlipConeCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private GamePiece prevGamePiece;
    private double queuedTime;
    private boolean isFinished = false;

    public FlipConeCommand(ArmStateMachine stateMachine) {
        this.stateMachine = stateMachine;
    }

    @Override
	public void initialize() {
        isFinished = false;
        prevGamePiece = stateMachine.getGamePiece();
        stateMachine.setGamePiece(GamePiece.CUBE);

        // Queued time used to distinguish running path from queued path if both are present
        queuedTime = Timer.getFPGATimestamp();

        stateMachine.pickup(ArmStateConstants.coneFlipFlexPosition, queuedTime);
	}

    @Override
    public void end(boolean interrupted) {
        stateMachine.handleCommandEnding(queuedTime);
        stateMachine.setGamePiece(prevGamePiece);
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
