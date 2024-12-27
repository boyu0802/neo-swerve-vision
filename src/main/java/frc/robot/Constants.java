package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class SwerveConstants {
        //TODO: get value
        public final static Rotation2d[] SWERVE_ANGLE_OFFSETS = {
            Rotation2d.fromRotations(0.634), //frontRight
            Rotation2d.fromRotations(0.018), // back Right
            Rotation2d.fromRotations(0.995), //back left
            Rotation2d.fromRotations(0.249) // front left
        };
        
        public static final double[] SWERVE_DRIVE_PID = {0.042, 0.0, 0.0}; //kp, ki, kd
        public static final double[] SWERVE_ANGLE_PID = {0.008, 0.0, 0.0}; 

        public static final double SWERVE_LENGTH = 0.75;
        public static final double SWERVE_WIDTH = 0.75;

        public static final double MAX_VOLTAGE = 12.0;

        public static final double KS = 0.32/MAX_VOLTAGE;
        public static final double KV = 1.51/MAX_VOLTAGE;
        public static final double KA = 0.27/MAX_VOLTAGE;

        public static final int FRONT_RIGHT_MODULE_ID = 0;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
        public static final int FRONT_RIGHT_CAN_CODER_ID = 2;

        public static final int BACK_RIGHT_MODULE_ID = 1;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 1;
        public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 2;
        public static final int BACK_RIGHT_CAN_CODER_ID = 1;

        public static final int BACK_LEFT_MODULE_ID = 2;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 7;
        public static final int BACK_LEFT_ANGLE_MOTOR_ID = 8;
        public static final int BACK_LEFT_CAN_CODER_ID = 4;

        public static final int FRONT_LEFT_MODULE_ID = 3;
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 6;
        public static final int FRONT_LEFT_CAN_CODER_ID = 3;


        public static final IdleMode DRIV_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;
        public static final boolean DRIVE_MOTOR_INVERTED = true;
        public static final boolean ANGLE_MOTOR_INVERTED = false;

        public static final double MAX_SPEED = 5.0;
        public static final double MAX_ANGULAR_SPEED = 6.7;
        public static final double SWERVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
        public static final int SWERVE_CURRENT_LIMIT = 35;


        public static final double DRIVE_POSITION_TO_METERS = (1.0/6.0) * SWERVE_WHEEL_CIRCUMFERENCE;
        public static final double DRIVE_POSITION_TO_MPS = DRIVE_POSITION_TO_METERS / 60.0;
        public static final double ANGLE_POSITION_TO_DEGREE = (1.0/20.0) * 360.0 ;

        public static final AbsoluteSensorRangeValue SENSOR_RANGE = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue SENSOR_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
        
        public static final boolean NAVX_INVERTED = true;

        public static final double SWERVE_DRIVE_JOYSTICK_DEADBAND = 0.05;


        public static final double VISION_AIM_KP = 1;
        public static final double VISION_AIM_KI = 0.0;
        public static final double VISION_AIM_KD = 0.0001;
        public static final String LIMELIGHT_NAME = "limelight-apr";
    }

   
}


