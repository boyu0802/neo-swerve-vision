package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsytem extends SubsystemBase {
    private SwerveModule[] modules;
    private AHRS Navx;
    
    public SwerveSubsytem() {
        modules = new SwerveModule[]{
            new SwerveModule(SwerveConstants.FRONT_RIGHT_MODULE_ID, SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, SwerveConstants.FRONT_RIGHT_ANGLE_MOTOR_ID, SwerveConstants.FRONT_RIGHT_CAN_CODER_ID, SwerveConstants.SWERVE_ANGLE_OFFSETS[0]),
            new SwerveModule(SwerveConstants.BACK_RIGHT_MODULE_ID, SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_ID, SwerveConstants.BACK_RIGHT_ANGLE_MOTOR_ID, SwerveConstants.BACK_RIGHT_CAN_CODER_ID, SwerveConstants.SWERVE_ANGLE_OFFSETS[1]),
            new SwerveModule(SwerveConstants.BACK_LEFT_MODULE_ID, SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID, SwerveConstants.BACK_LEFT_ANGLE_MOTOR_ID, SwerveConstants.BACK_LEFT_CAN_CODER_ID ,SwerveConstants.SWERVE_ANGLE_OFFSETS[2]),
            new SwerveModule(SwerveConstants.FRONT_LEFT_MODULE_ID, SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID, SwerveConstants.FRONT_LEFT_ANGLE_MOTOR_ID,SwerveConstants.FRONT_LEFT_CAN_CODER_ID ,SwerveConstants.SWERVE_ANGLE_OFFSETS[3])
        };

        Navx = new AHRS(SPI.Port.kMXP);
        Navx.reset();

        SwerveDriveOdometry odometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_DRIVE_KINEMATICS, Navx.getRotation2d(),getModulePositions());
        Timer.delay(1.0);
        resetModulesToAbsolute();
    }


}
