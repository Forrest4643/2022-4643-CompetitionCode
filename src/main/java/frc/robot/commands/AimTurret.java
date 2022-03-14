// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretPIDSubsystem;
//import frc.robot.subsystems.VisionSubsystem;

public class AimTurret extends CommandBase {
  /** Creates a new TurretPosition. */
  //private final VisionSubsystem visionSubsystem;
  private final TurretPIDSubsystem turretPIDSubsystem;
  private final double turretSetpoint;
  public AimTurret(TurretPIDSubsystem turretPIDSubsystem, /*VisionSubsystem visionSubsystem,*/ double turretSetpoint) {
    //this.visionSubsystem = visionSubsystem;
    this.turretPIDSubsystem = turretPIDSubsystem;
    this.turretSetpoint = turretSetpoint;
    addRequirements(turretPIDSubsystem/*, visionSubsystem*/);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurretPosition Started!");
  }

  // public double turretSetpoint() {
  //     return visionSubsystem.getTargetYaw() + turretPIDSubsystem.turretPosition();
  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      turretPIDSubsystem.setSetpoint(turretSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("TurretPosition Started!");
    turretPIDSubsystem.setSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
