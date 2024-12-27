// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.SwerveSubsytem;
import frc.robot.command.SwerveCommand;

public class RobotContainer {
  private final SwerveSubsytem swerveSubsystem = new SwerveSubsytem();
  private final XboxController driveController = new XboxController(0);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveCommand(
                swerveSubsystem,
                () -> driveController.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> driveController.getRawAxis(XboxController.Axis.kLeftX.value),
                () -> driveController.getRightX(),
                () -> driveController.getLeftBumper(),
                () -> driveController.getAButton()));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driveController,XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
