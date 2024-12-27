package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.ModuleState;

public class SwerveModule{
    public int moduleID;
    private int driveMotorID;
    private int angleMotorID;
    private int canCoderID;
    private Rotation2d angleOffset;
    
    private CANSparkFlex driveMotor;
    private CANSparkMax angleMotor;
    private CANcoder canCoder;

    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(SwerveConstants.KS, SwerveConstants.KV, SwerveConstants.KA);

    public SwerveModule(int moduleID, int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset){
        this.moduleID = moduleID;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleOffset = angleOffset;
        this.canCoderID = canCoderID;

        driveMotor = new CANSparkFlex(driveMotorID,MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        canCoder = new CANcoder(canCoderID);

        configDrive();
        configAngle();
        configCanCoder();
        resetToAbsolute();

    }
    
    
    public void setModuleState(SwerveModuleState state, boolean isOpenLoop){
        SwerveModuleState desiredState = ModuleState.optimiize(state, getModuleState().angle);

        
        SmartDashboard.putNumber( "mod" + moduleID + "module state angle...", desiredState.angle.getDegrees());
        SmartDashboard.putNumber( "mod" + moduleID + "ange motor angle", angleMotor.getEncoder().getPosition());

        setAngle(desiredState);
        setSpeed(desiredState,isOpenLoop);
    }

    public void setAngle(SwerveModuleState state){
        angleMotor.getPIDController().setReference(state.angle.getDegrees(),ControlType.kPosition,0);
    }

    public void setSpeed(SwerveModuleState state, boolean isOpenLoop){
        if(isOpenLoop){
            double speed = state.speedMetersPerSecond/ SwerveConstants.MAX_SPEED;
            driveMotor.set(speed);
        }else{
            driveMotor.getPIDController().setReference(state.speedMetersPerSecond, ControlType.kVelocity,0,ff.calculate(state.speedMetersPerSecond));
        }
    }

    public double getDriveMPS(){
        return driveMotor.getEncoder().getVelocity();
    }

    public double getDriveMeters(){
        return driveMotor.getEncoder().getPosition();
    }

    public Rotation2d getEncoderAngle(){
        return Rotation2d.fromDegrees(angleMotor.getEncoder().getPosition());
    }
    public Rotation2d getCancoderAngle(){
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValue());
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveMPS(),getEncoderAngle());
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(getDriveMeters(), getEncoderAngle());
    }

    public void resetToAbsolute(){
        double angle = (getCancoderAngle().getDegrees() - angleOffset.getDegrees());
        angleMotor.getEncoder().setPosition(angle);
    }


    

    private void configDrive(){
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(SwerveConstants.DRIVE_MOTOR_INVERTED);
        driveMotor.setIdleMode(SwerveConstants.DRIV_IDLE_MODE);
        driveMotor.setSmartCurrentLimit(SwerveConstants.SWERVE_CURRENT_LIMIT);
        driveMotor.getEncoder().setPositionConversionFactor(SwerveConstants.DRIVE_POSITION_TO_METERS);
        driveMotor.getEncoder().setVelocityConversionFactor(SwerveConstants.DRIVE_POSITION_TO_MPS);
        driveMotor.getPIDController().setP(SwerveConstants.SWERVE_DRIVE_PID[0]);
        driveMotor.getPIDController().setI(SwerveConstants.SWERVE_DRIVE_PID[1]);
        driveMotor.getPIDController().setD(SwerveConstants.SWERVE_DRIVE_PID[2]);
        driveMotor.burnFlash();
    }

    private void configAngle(){
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(SwerveConstants.ANGLE_MOTOR_INVERTED);
        angleMotor.setIdleMode(SwerveConstants.ANGLE_IDLE_MODE);
        angleMotor.setSmartCurrentLimit(SwerveConstants.SWERVE_CURRENT_LIMIT);
        angleMotor.getEncoder().setPositionConversionFactor(SwerveConstants.ANGLE_POSITION_TO_DEGREE);
        angleMotor.getPIDController().setP(SwerveConstants.SWERVE_ANGLE_PID[0]);
        angleMotor.getPIDController().setI(SwerveConstants.SWERVE_ANGLE_PID[1]);
        angleMotor.getPIDController().setD(SwerveConstants.SWERVE_ANGLE_PID[2]);
        angleMotor.burnFlash();
    }

    public void configCanCoder(){
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = SwerveConstants.SENSOR_RANGE;
        canCoderConfig.MagnetSensor.SensorDirection = SwerveConstants.SENSOR_DIRECTION;
        canCoder.getConfigurator().apply(canCoderConfig);
    }

}

