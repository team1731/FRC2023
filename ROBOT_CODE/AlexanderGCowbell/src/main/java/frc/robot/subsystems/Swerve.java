package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import frc.robot.SwerveModule;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  

    public SwerveModule[] mSwerveMods;
  //  public PigeonIMU gyro;
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
    public Double lockedHeading = null;
    private boolean s_lockWheels = false;

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

 
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        if (s_lockWheels && 
            (swerveModuleStates[0].speedMetersPerSecond == 0.0 &&
            swerveModuleStates[1].speedMetersPerSecond == 0.0 &&
            swerveModuleStates[2].speedMetersPerSecond == 0.0 &&
            swerveModuleStates[3].speedMetersPerSecond == 0.0 )
            ) {
            swerveModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0))};
        }

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop, s_lockWheels);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
           
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false, false);
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
	public Rotation2d getHeading() {	
		double m_heading = 0.0;
        if (m_gyro != null) {
			m_heading = Math.IEEEremainder(m_gyro.getAngle(), 360) * -1.0;
            if(Robot.doSD()){
                SmartDashboard.putNumber("Gyro -> HEADING", m_heading);

            }
		}
        return Rotation2d.fromDegrees(m_heading);

	}

    public void zeroGyro(){

        lockedHeading = null;
        m_gyro.reset();
       // m_gyro.zeroYaw();
 
    }

    public void adjustWheelEncoders(){
              for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
            System.out.println("Adjusting Wheel Encoders!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }  
    }

    public double getPitch() {
        return m_gyro.getRoll(); // pitch is roll given the way the board is mounted.
    }

  //  public Rotation2d getYaw() {
  //  if (m_gyro.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
  //    return Rotation2d.fromDegrees(m_gyro.getFusedHeading());
  //  }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   // return Rotation2d.fromDegrees(360.0 - m_gyro.getYaw());
   // }

    @Override
    public void periodic(){
    //    SmartDashboard.putNumber("Yaw in degrees" , getYaw().getDegrees()) ; 
        SmartDashboard.putNumber("getHeading",getHeading().getDegrees());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().toString());
        SmartDashboard.putBoolean("Swerve Locked", s_lockWheels);
 
        if (Robot.doSD()) {
            for(SwerveModule mod : mSwerveMods){
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);  

            }
        }
    }

    public void setLockWheels(boolean b) {
        s_lockWheels = b;
    }
}