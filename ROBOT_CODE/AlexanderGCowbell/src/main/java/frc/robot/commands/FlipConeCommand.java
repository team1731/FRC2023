package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.ArmStateConstants;
import frc.robot.state.arm.ArmStateMachine;

public class FlipConeCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private GamePiece prevGamePiece;

    public FlipConeCommand(ArmStateMachine stateMachine) {
        this.stateMachine = stateMachine;
    }

    @Override
	public void initialize() {
        prevGamePiece = stateMachine.getGamePiece();
        stateMachine.setGamePiece(GamePiece.CUBE);
        stateMachine.pickup(ArmStateConstants.coneFlipFlexPosition);
	}

    @Override
    public void end(boolean interrupted) {
        stateMachine.buttonReleased();
        stateMachine.setGamePiece(prevGamePiece);
    }
}
