package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.data.mp.*;

public class ArmScoreTestCommand extends CommandBase {
    private ArmSubsystem armSubsystem;

    public ArmScoreTestCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
	public void initialize() {
        ArmPath path = ScoreTest.getArmPath();
        armSubsystem.startArmMovement(path);
	}

    @Override
    public void end(boolean interrupted) {
        armSubsystem.eject();
        armSubsystem.reverseArmMovment();
    }
}
