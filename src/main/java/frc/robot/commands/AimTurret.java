// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimTurret extends CommandBase {
  /** Creates a new TurretPosition. */
  private final VisionSubsystem visionSubsystem;
  private final TurretPIDSubsystem turretPIDSubsystem;
  private final BooleanSupplier m_aiming;

  public AimTurret(BooleanSupplier aiming, TurretPIDSubsystem turretPIDSubsystem, VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.turretPIDSubsystem = turretPIDSubsystem;
    m_aiming = aiming;
    addRequirements(turretPIDSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurretPosition Started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_aiming.getAsBoolean()) {
      turretPIDSubsystem.setSetpoint(visionSubsystem.getTargetYaw() + turretPIDSubsystem.turretPosition());
    } else {
      turretPIDSubsystem.disable();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("TurretPosition Started!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
