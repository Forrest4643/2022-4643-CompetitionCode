// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.commands.TurretOLD;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretTrackTarget extends PIDCommand {
  /** Creates a new TurretTrackTarget. */
  public TurretTrackTarget(TurretSubsystem turretSubsystem, DoubleSupplier targetYaw) {
    super(
        // The controller that the command will use
        new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD),
        // This should return the measurement
        () -> targetYaw.getAsDouble(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          output = MathUtil.clamp(output, -1, 1);
          turretSubsystem.setTurretSpeed(output);
        });
    addRequirements(turretSubsystem);
    getController().enableContinuousInput(-135, 135);
    getController().setTolerance(0.1);

  }

  @Override
  public void initialize() {
    System.out.println("TurretTrackTarget Started!");
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("TurretTrackTarget Ended!");
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
