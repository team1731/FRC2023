package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

	private double driveSpeedScaler = 1.0;
    private boolean headingOverride = true;
	private boolean visionHeadingOverride = false;
    private Double lockedHeading = null;
    private Double desiredHeading;
	private double m_heading;
    
    private final ProfiledPIDController headingController = 
        new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD,
        new TrapezoidProfile.Constraints(VisionConstants.kMaxTurnVelocity, VisionConstants.kMaxTurnAcceleration));

    public SwerveModule[] mSwerveMods;
//  public PigeonIMU gyro;
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP


    public Swerve() {

        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        adjustWheelEncoders();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        boolean useLockHeadingCode = false;

        if(useLockHeadingCode){ //TODO FIXME: debug this block!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            double xSpeedAdjusted = translation.getX();
            double ySpeedAdjusted = translation.getY();
    
            double rotationalOutput = rotation;
    
            // DEADBAND
            if (Math.abs(xSpeedAdjusted) < 0.1) {
                xSpeedAdjusted = 0;
            }
            if (Math.abs(ySpeedAdjusted) < 0.1) {
                ySpeedAdjusted = 0;
            }
            xSpeedAdjusted *= this.driveSpeedScaler;
            ySpeedAdjusted *= this.driveSpeedScaler;
    
    
            // If the right stick is neutral - this code should lock onto the last known
            // heading
            if (Math.abs(rotationalOutput) < 0.11) {
                headingOverride = true;
                if (lockedHeading == null) {
                    headingController.reset(getHeading());
                    desiredHeading = getHeading();
                    lockedHeading = desiredHeading;
                } else {
                    desiredHeading = lockedHeading;
                }
            } else {
                headingOverride = false;
                lockedHeading = null;
                rotationalOutput *= Math.PI;
            }
    
            if (visionHeadingOverride || headingOverride) {
    
                if (visionHeadingOverride) {
                    rotationalOutput = headingController.calculate(getHeading());
                    desiredHeading = getHeading();
                    lockedHeading = getHeading();
                    SmartDashboard.putNumber("headingController Output", rotationalOutput);
                } else {
                    // headingController.reset(getHeading());
                    // desiredHeading += rotationalOutput*2.5;
                    rotationalOutput = headingController.calculate(getHeading(), desiredHeading);
                    SmartDashboard.putNumber("desiredHeading", desiredHeading);
                    SmartDashboard.putNumber("headingController Output", rotationalOutput);
                }
            }
        }

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {	
		if (m_gyro != null) {
			m_heading = Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
			if (System.currentTimeMillis() % 100 == 0) {
				SmartDashboard.putNumber("Heading", m_heading);
			}
		}
		return m_heading;
	}

    public void zeroGyro(){
        m_gyro.zeroYaw();
    }

    public void adjustWheelEncoders(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
            DataLogManager.log("Adjusting Wheel Encoders!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }  
    }

    public Rotation2d getYaw() {
        if (m_gyro.isMagnetometerCalibrated()) {
        // We will only get valid fused headings if the magnetometer is calibrated
        return Rotation2d.fromDegrees(m_gyro.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_gyro.getYaw());
    }

    @Override
    public void periodic(){
        if (Robot.doSD()) {
            for(SwerveModule mod : mSwerveMods){
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
            }
        }
    }
}