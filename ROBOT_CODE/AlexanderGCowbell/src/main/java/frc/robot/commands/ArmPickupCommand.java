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
    private Joystick joystick;
    private int distalAxis;
    private boolean adjustWrist = false;

    public ArmPickupCommand(ArmStateMachine stateMachine, ArmSequence sequence, Joystick joystick, int distalAxis) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
        this.joystick = joystick;
        this.distalAxis = distalAxis;
    }

    @Override
	public void initialize() {
        if(stateMachine.getStatus() == Status.RUNNING) {
            System.out.println("WARNING: cannot command a pickup when arm state is already running a movement");
            return;
        }

        // no movement is currently running, load the correct path and start the pickup
        ArmPath path = null;
        boolean shouldDoExtraExtension = stateMachine.getExtraExtension();
        if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CONE && shouldDoExtraExtension) {
            path = PickupHighConeExtra.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CONE && !shouldDoExtraExtension) {
            path = PickupHighCone.getArmPath();
        } if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CUBE && shouldDoExtraExtension) {
            path = PickupHighCubeExtra.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CUBE && !shouldDoExtraExtension) {
            path = PickupHighCube.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupLowCone.getArmPath();
            adjustWrist = true;
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CUBE) {
            stateMachine.pickup(ArmStateConstants.pickupLowCubeFlexPosition);
            adjustWrist = true;
        } else if(sequence == ArmSequence.PICKUP_LOW_CUBE) {
            // can be fired individually, also used to flip downed cone
            stateMachine.pickup(ArmStateConstants.pickupLowCubeFlexPosition);
            stateMachine.setJoystickControl(joystick, distalAxis, adjustWrist);
            adjustWrist = true;
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
