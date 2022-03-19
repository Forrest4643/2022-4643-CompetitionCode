// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class hoodPID extends PIDCommand {
  /** Creates a new turretPID. */
  public hoodPID(HoodSubsystem hoodSubsystem, DoubleSupplier hoodPositionIN) {
    super(
        // The controller that the command will use
        new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD),
        // This should return the measurement
        () -> hoodSubsystem.getHoodPositionIN(),
        // This should return the setpoint (can also be a constant)
        () -> hoodPositionIN.getAsDouble(),
        // This uses the output
        output -> {
          ElevatorFeedforward hFeedforward = new ElevatorFeedforward(HoodConstants.kS, HoodConstants.kG,
              HoodConstants.kV);
          output = MathUtil.clamp(output, -.3, .3);
          hoodSubsystem.setHoodMotor(output + hFeedforward.calculate(hoodSubsystem.getHoodVelocity()));
        });
    addRequirements(hoodSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
