package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmSequence;
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
        stateMachine.setIsInAuto(true);
    }

    @Override
	public void initialize() {
        if(stateMachine.getStatus() == Status.RUNNING) {
            System.out.println("WARNING: autonomous cannot command a pickup when arm state is already running a movement");
            return;
        }

        ArmPath path = null;
        if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupHighCone.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = PickupHighCube.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupLowCone.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = PickupLowCube.getArmPath();
        }

        if(path != null) {
            stateMachine.pickup(path);
        } else {
            isFinished = true;
        }
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
