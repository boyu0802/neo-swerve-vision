package frc.robot.command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsytem;
import frc.robot.Constants.SwerveConstants;

public class SwerveCommand extends Command{
    private SwerveSubsytem swerveSubsystem;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldRelativeSup;
    private BooleanSupplier openLoopSup;

    public SwerveCommand(SwerveSubsytem swerveSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier fieldRelativeSup, BooleanSupplier openLoopSup) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldRelativeSup = fieldRelativeSup;
        this.openLoopSup = openLoopSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), SwerveConstants.SWERVE_DRIVE_JOYSTICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveConstants.SWERVE_DRIVE_JOYSTICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SwerveConstants.SWERVE_DRIVE_JOYSTICK_DEADBAND);
        

        /* Drive */
        swerveSubsystem.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED),
                rotationVal * SwerveConstants.MAX_ANGULAR_SPEED,
                fieldRelativeSup.getAsBoolean(),
                openLoopSup.getAsBoolean()
        );
    }
}

