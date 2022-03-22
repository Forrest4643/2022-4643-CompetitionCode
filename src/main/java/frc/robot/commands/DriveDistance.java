// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {

  private DriveSubsystem driveSubsystem;
  private double distance;
  private PIDController drivePID = new PIDController(DriveConstants.drivekP, DriveConstants.drivekI, DriveConstants.drivekD);
  /** Creates a new DriveDistance. */
  public DriveDistance(DriveSubsystem driveSubsystem, double distance) {
    this.driveSubsystem = driveSubsystem;
    this.distance = distance;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID.setSetpoint(driveSubsystem.getDriveDistanceFT() + distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setDrive(drivePID.calculate(driveSubsystem.getDriveDistanceFT()), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePID.atSetpoint();
  }
}
