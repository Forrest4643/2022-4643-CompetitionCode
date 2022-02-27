// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurretPosition extends CommandBase {
  /** Creates a new TurretPosition. */
  private final VisionSubsystem visionSubsystem;
  private final TurretPIDSubsystem turretPIDSubsystem;
  private double m_setpoint; 
  public TurretPosition(TurretPIDSubsystem turretPIDSubsystem, VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.turretPIDSubsystem = turretPIDSubsystem;
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
    m_setpoint = visionSubsystem.getTargetYaw() + turretPIDSubsystem.turretPosition();
    turretPIDSubsystem.setSetpoint(m_setpoint);
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
