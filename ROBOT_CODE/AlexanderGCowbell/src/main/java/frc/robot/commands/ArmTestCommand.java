package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private boolean started = false;

    public ArmTestCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
	public void initialize() {
        armSubsystem.initialize();
        armSubsystem.initializeArmMovement();
        armSubsystem.moveArm();   
        started = true;     
	}

    @Override
    public boolean isFinished() {
        return started && !armSubsystem.isArmMoving();
    }
}
