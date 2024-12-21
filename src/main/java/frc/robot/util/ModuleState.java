package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleState  {
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double targetAngle = placeInAppropriateRange(desiredState.angle.getDegrees(), currentAngle.getDegrees());
        double targetVelocity = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if(Math.abs(delta) > 90){
            targetVelocity = -targetVelocity;
            targetAngle = (targetAngle > 90) ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetVelocity, Rotation2d.fromDegrees(targetAngle));
    }

    public static double placeInAppropriateRange(double desiredAngle, double currentAngle){
        double upperBound;
        double lowerBound;
        double angle = currentAngle % 360;
        if(angle < 0){
            upperBound = currentAngle - angle;
            lowerBound = currentAngle - angle - 360;
        }
        else{
            lowerBound = currentAngle - angle;
            upperBound = currentAngle - angle + 360;
        }
        while(desiredAngle > upperBound) {desiredAngle -= 360;}
        while(desiredAngle < lowerBound) {desiredAngle += 360;}
        if(desiredAngle - currentAngle > 180)
            desiredAngle -= 360;
        else if(desiredAngle - currentAngle < -180)
            desiredAngle += 360;
        return desiredAngle;
    }
}
