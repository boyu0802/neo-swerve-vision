package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class SwerveConstants {
        //TODO: get value
        public final static Rotation2d[] SWERVE_ANGLE_OFFSETS = {
            Rotation2d.fromRotations(0.01), //frontRight
            Rotation2d.fromRotations(0.5), // back Right
            Rotation2d.fromRotations(0.3), //back left
            Rotation2d.fromRotations(1) // front left
        };
        
        public static final double[] SWERVE_DRIVE_PID = {0.0, 0.0, 0.0};
        public static final double[] SWERVE_ANGLE_PID = {0.0, 0.0, 0.0};

        public static final double SWERVE_LENGTH = 0.0;
        public static final double SWERVE_WIDTH = 0.0;

        public static final double MAX_VOLTAGE = 12.0;

        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;

        public static final IdleMode DRIV_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean ANGLE_MOTOR_INVERTED = false;

        public static final double MAX_SPEED = 4.7;
        public static final double MAX_ANGULAR_SPEED = 4.7;
        public static final double SWERVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
        public static final int SWERVE_CURRENT_LIMIT = 40;

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(SWERVE_LENGTH / 2, SWERVE_WIDTH / 2), //front right 
            new Translation2d(-SWERVE_LENGTH / 2, SWERVE_WIDTH / 2), //back right
            new Translation2d(-SWERVE_LENGTH / 2, -SWERVE_WIDTH / 2), //back left
            new Translation2d(SWERVE_LENGTH / 2, -SWERVE_WIDTH / 2) //  front left
        );

        public static final double DRIVE_POSITION_TO_METERS = (1.0/6.0) * SWERVE_WHEEL_CIRCUMFERENCE;
        public static final double DRIVE_POSITION_TO_MPS = DRIVE_POSITION_TO_METERS / 60.0;
        public static final double ANGLE_POSITION_TO_DEGREE = (1.0/20.0) * 360.0 ;

        public static final AbsoluteSensorRangeValue SENSOR_RANGE = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue SENSOR_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    }
}


