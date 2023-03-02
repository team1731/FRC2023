package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashSet;
import java.util.Set;


public class TestWheelLockCommand extends CommandBase {
    
    private Swerve swerve;

    /**
     * Driver control
     */
    public TestWheelLockCommand(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> subsystems = new HashSet<Subsystem>();
        subsystems.add(swerve);
        return subsystems; 
    }

    @Override
    public void execute() {
        swerve.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0.0, new Rotation2d(45.0)),
            new SwerveModuleState(0.0, new Rotation2d(315.0)),
            new SwerveModuleState(0.0, new Rotation2d(135.0)),
            new SwerveModuleState(0.0, new Rotation2d(225.0))
        });
    }
}
