// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class driveMoveTarget extends PIDCommand {
  /** Creates a new driveAimTarget. */
  public driveMoveTarget(driveAim driveAim, VisionSubsystem visionSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.drivekP, DriveConstants.drivekI, DriveConstants.drivekD),
        // This should return the measurement
        () -> visionSubsystem.getTargetDistance(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          driveAim.setDrive(output);
        });
    getController().setTolerance(3);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
