package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private boolean headingOverride = true;
	private boolean visionHeadingOverride = false;
    private Double lockedHeading = null;
    private Double desiredHeading;
	private double m_heading;
    
    private final ProfiledPIDController headingController = 
        new ProfiledPIDController(VisionConstants.kTurnP, VisionConstants.kTurnI, VisionConstants.kTurnD,
        new TrapezoidProfile.Constraints(VisionConstants.kMaxTurnVelocity, VisionConstants.kMaxTurnAcceleration));

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private XboxController controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis) ;
        double xAxis = -controller.getRawAxis(strafeAxis) ;
        double rAxis = -controller.getRawAxis(rotationAxis) ;
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : (yAxis - Constants.stickDeadband)*Math.abs(yAxis - Constants.stickDeadband);
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : (xAxis- Constants.stickDeadband)*Math.abs(yAxis - Constants.stickDeadband);
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : (rAxis- Constants.stickDeadband)*Math.abs(yAxis - Constants.stickDeadband);

        translation = new Translation2d(yAxis , xAxis).times(Constants.Swerve.maxSpeed);


            // If the right stick is neutral - this code should lock onto the last known
            // heading
            if (Math.abs(rAxis) == 0) {
                headingOverride = true;
                if (lockedHeading == null) {
                    headingController.reset(s_Swerve.getHeading());
                    desiredHeading = s_Swerve.getHeading();
                    lockedHeading = desiredHeading;
                } else {
                    desiredHeading = lockedHeading;
                }
                rotation = headingController.calculate(s_Swerve.getHeading(), desiredHeading);
            } else {
                headingOverride = false;
                lockedHeading = null;
                rotation = rAxis * Constants.Swerve.maxAngularVelocity;
            }
    
          
   




        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
