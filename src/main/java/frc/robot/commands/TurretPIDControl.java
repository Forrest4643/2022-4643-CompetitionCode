// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretPIDControl extends PIDCommand {
  /** Creates a new TurretPIDControl. */
  public TurretPIDControl(TurretSubsystem turretSubsystem, DoubleSupplier setpoint) {
    super(
        // The controller that the command will use
        new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD),
        // This should return the measurement
        () -> turretSubsystem.turretPositionDEG(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint.getAsDouble(),
        // This uses the output
        output -> {
        turretSubsystem.setTurretSpeed(output);
        });
        addRequirements(turretSubsystem);

        getController().setTolerance(0);
    // Configure additional PID options by calling `getController` here.
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
