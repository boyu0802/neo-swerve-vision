package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.SwerveModule;

public class SwerveSubsytem extends SubsystemBase {
    private SwerveModule[] modules;
    private AHRS Navx;
    private SwerveDriveOdometry odometry;
    
    public SwerveSubsytem() {
        modules = new SwerveModule[]{
            new SwerveModule(SwerveConstants.FRONT_RIGHT_MODULE_ID, SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, SwerveConstants.FRONT_RIGHT_ANGLE_MOTOR_ID, SwerveConstants.FRONT_RIGHT_CAN_CODER_ID, SwerveConstants.SWERVE_ANGLE_OFFSETS[0]),
            new SwerveModule(SwerveConstants.BACK_RIGHT_MODULE_ID, SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_ID, SwerveConstants.BACK_RIGHT_ANGLE_MOTOR_ID, SwerveConstants.BACK_RIGHT_CAN_CODER_ID, SwerveConstants.SWERVE_ANGLE_OFFSETS[1]),
            new SwerveModule(SwerveConstants.BACK_LEFT_MODULE_ID, SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID, SwerveConstants.BACK_LEFT_ANGLE_MOTOR_ID, SwerveConstants.BACK_LEFT_CAN_CODER_ID ,SwerveConstants.SWERVE_ANGLE_OFFSETS[2]),
            new SwerveModule(SwerveConstants.FRONT_LEFT_MODULE_ID, SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID, SwerveConstants.FRONT_LEFT_ANGLE_MOTOR_ID,SwerveConstants.FRONT_LEFT_CAN_CODER_ID ,SwerveConstants.SWERVE_ANGLE_OFFSETS[3])
        };

        Navx = new AHRS(SPI.Port.kMXP);
        Navx.reset();

        odometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_DRIVE_KINEMATICS, Navx.getRotation2d(),getModulePositions());
        Timer.delay(1.0);
        resetModulesToAbsolute();
    }

        public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

        for(SwerveModule mod : modules){
            mod.setModuleState(swerveModuleStates[mod.moduleID], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

        for(SwerveModule mod : modules){
            mod.setModuleState(desiredStates[mod.moduleID], false);
        }
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : modules){
            states[mod.moduleID] = mod.getModuleState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : modules){
            positions[mod.moduleID] = mod.getModulePosition();
        }
        return positions;
    }

    public void zeroGyro(){
        Navx.reset();
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.NAVX_INVERTED) ? Rotation2d.fromDegrees(360 - Navx.getYaw()) : Rotation2d.fromDegrees(Navx.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule module : modules){
            module.resetToAbsolute();
        }
    }


    @Override
    public void periodic(){
        odometry.update(getYaw(), getModulePositions());

        for(SwerveModule mod : modules){
            SmartDashboard.putNumber("Mod " + mod.moduleID + " Cancoder", mod.getCancoderAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleID + " Integrated", mod.getModulePosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleID + " Velocity", mod.getModuleState().speedMetersPerSecond);
        }
    }
    


}
