package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.ArmStateConstants;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.Status;
import frc.data.mp.*;

public class ArmPickupCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;

    public ArmPickupCommand(ArmStateMachine stateMachine, ArmSequence sequence, Joystick joystick, int distalAxis) {
        this.stateMachine = stateMachine;
        this.stateMachine.setJoystickControl(joystick, distalAxis);
        this.sequence = sequence;
    }

    @Override
	public void initialize() {
        if(stateMachine.getStatus() == Status.RUNNING) {
            System.out.println("WARNING: cannot command a pickup when arm state is already running a movement");
            return;
        }

        // no movement is currently running, load the correct path and start the pickup
        ArmPath path = null;
        if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupHighCone.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = PickupHighCube.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupLowCone.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CUBE) {
            stateMachine.pickup(ArmStateConstants.pickupLowCubeFlexPosition);
        }

        if(path != null) {
            stateMachine.pickup(path);
        }
	}

    @Override
    public void end(boolean interrupted) {
        stateMachine.buttonReleased();
    }
}
