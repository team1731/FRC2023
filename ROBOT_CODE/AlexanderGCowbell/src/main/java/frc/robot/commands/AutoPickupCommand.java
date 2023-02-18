package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmState;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.Status;
import frc.data.mp.*;

public class AutoPickupCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;
    private boolean started = false;
    private boolean isFinished = false;

    public AutoPickupCommand(ArmStateMachine stateMachine, ArmSequence sequence, GamePiece pieceType) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
        stateMachine.setGamePiece(pieceType);
    }

    @Override
	public void initialize() {
        ArmPath path = null;
        if(sequence == ArmSequence.PICKUP_HIGH) {
            path = PickupHigh.getArmPath();
        } else if(stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupLowCone.getArmPath();
        } else {
            path = PickupLowCube.getArmPath();
        }

        stateMachine.pickup(path);
	}

    @Override
    public void execute() {
        if(!started && stateMachine.getStatus() == Status.RUNNING) {
            started = true;
        } else if(started && stateMachine.getStatus() == Status.READY) {
            // has returned to a ready state, we are done
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
