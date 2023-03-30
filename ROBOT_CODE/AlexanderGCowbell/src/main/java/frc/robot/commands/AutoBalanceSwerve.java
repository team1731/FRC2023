package frc.robot.commands;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoBalanceSwerve extends CommandBase {


    private Double desiredHeading = 0.0;
    private boolean fieldrelative = true;
    private boolean openLoop = false;
    private boolean isFinished = false;

    
    private final PIDController headingController = 
        new PIDController(VisionConstants.kTurnP, VisionConstants.kTurnI, VisionConstants.kTurnD);
       

    private double rotation;
    private Translation2d translation = new Translation2d(0 , 0);

    
    private Swerve s_Swerve;

    /**
     * Driver control
     */
    public AutoBalanceSwerve(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);  
    }
  
    @Override
    public void initialize() {
        isFinished = false;
        s_Swerve.setLockWheels(true);
    }
    @Override
    public void execute() {

 
        rotation = headingController.calculate(s_Swerve.getHeading().getDegrees(), desiredHeading);  
        if (s_Swerve.getPitch() > 9) {
            translation = new Translation2d(-0.25, 0.0); // Speed is in Meters/s
        } else if (s_Swerve.getPitch() < -9) {
            translation = new Translation2d(0.25, 0);
        } else {
            translation = new Translation2d(0 , 0);
            if(Timer.getMatchTime() <= 0.1) {
                s_Swerve.setLockWheels(false);
                isFinished = true;
            }
        }

        s_Swerve.drive(translation, rotation, fieldrelative, openLoop);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean innterruped) {
        s_Swerve.setLockWheels(false);
        isFinished = true;
    }
}
