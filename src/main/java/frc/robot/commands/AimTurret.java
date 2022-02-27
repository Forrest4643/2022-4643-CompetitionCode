// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretPIDSubsystem;



public class AimTurret extends CommandBase {
  /** Creates a new AimTurret. */
  private double setpoint;

  private final TurretPIDSubsystem turretPIDSubsystem;

  public AimTurret(TurretPIDSubsystem turretPIDSubsystem) {
    this.turretPIDSubsystem = turretPIDSubsystem;
    addRequirements(turretPIDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AimTurret Started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AimTurret Ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
