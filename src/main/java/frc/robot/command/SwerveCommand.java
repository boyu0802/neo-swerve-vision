package frc.robot.command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsytem;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;

public class SwerveCommand extends Command{
    private SwerveSubsytem swerveSubsystem;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldRelativeSup;
    private BooleanSupplier openLoopSup;

    private BooleanSupplier visionAimSup;
    private final PIDController VISION_AIM_PID = new PIDController(SwerveConstants.VISION_AIM_KP, SwerveConstants.VISION_AIM_KI, SwerveConstants.VISION_AIM_KD);
    
    public SwerveCommand(SwerveSubsytem swerveSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier fieldRelativeSup, BooleanSupplier visionAimSup,BooleanSupplier openLoopSup) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldRelativeSup = fieldRelativeSup;
        this.visionAimSup = visionAimSup;
        this.openLoopSup = openLoopSup;

        VISION_AIM_PID.setTolerance(0.2);
        //VISION_AIM_PID.setIntegratorRange(-0.5, 0.5);
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), SwerveConstants.SWERVE_DRIVE_JOYSTICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveConstants.SWERVE_DRIVE_JOYSTICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SwerveConstants.SWERVE_DRIVE_JOYSTICK_DEADBAND);
        double visionAimNum = LimelightHelpers.getTX(SwerveConstants.LIMELIGHT_NAME);
        //SmartDashboard.putBoolean("vision",visionAimSup.getAsBoolean());
        SmartDashboard.putNumber("tx",LimelightHelpers.getTX("limelight-apr"));
        if(visionAimSup.getAsBoolean() && visionAimNum!=0.0){
            double visionRot = VISION_AIM_PID.calculate(visionAimNum, 0);
            swerveSubsystem.drive(
                new Translation2d(translationVal,strafeVal).times(SwerveConstants.MAX_SPEED),
                visionRot * -SwerveConstants.MAX_ANGULAR_SPEED,
                fieldRelativeSup.getAsBoolean(),
                openLoopSup.getAsBoolean());
        }
        
        else { 
        swerveSubsystem.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED),
            rotationVal * SwerveConstants.MAX_ANGULAR_SPEED,
            fieldRelativeSup.getAsBoolean(),
            false);
        }

    }
}

