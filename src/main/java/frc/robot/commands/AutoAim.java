// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private ShooterPIDSubsystem shooterPIDSubsystem;
  private HoodPIDSubsystem hoodPIDSubsystem;
  private IndexerSubsystem indexerSubsystem;

  /** Creates a new driveAim. */
  public AutoAim(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      ShooterPIDSubsystem shooterPIDSubsystem, HoodPIDSubsystem hoodPIDSubsystem, IndexerSubsystem indexerSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.shooterPIDSubsystem = shooterPIDSubsystem;
    this.hoodPIDSubsystem = hoodPIDSubsystem;

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    System.out.println("AutoAim Started!");
    addRequirements(driveSubsystem, hoodPIDSubsystem, shooterPIDSubsystem);
    shooterPIDSubsystem.enable();
    hoodPIDSubsystem.enable();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double targetDistance = visionSubsystem.getTargetDistanceFT();
    double targetYaw = visionSubsystem.getTargetYaw();

    // aiming hood
    hoodPIDSubsystem.setSetpoint(MathUtil.clamp(
        HoodConstants.quadAimC +
            (HoodConstants.quadAimB * targetDistance) +
            (Math.pow(targetDistance, 2) * HoodConstants.quadAimA),

        65, 80));

    // setting shooter RPM
    shooterPIDSubsystem.setSetpoint(
        ShooterConstants.quadAimD +
            (ShooterConstants.quadAimC * targetDistance) +
            (Math.pow((targetDistance), 2) * ShooterConstants.quadAimB) +
            (Math.pow(targetDistance, 3) * ShooterConstants.quadAimA));

    SmartDashboard.putNumber("shooterRPM", shooterPIDSubsystem.getShooterRPM());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterPIDSubsystem.disable();
    hoodPIDSubsystem.disable();
    driveSubsystem.setDrive(0, 0);
    System.out.println("AutoAim Ended!");
  }

  public boolean readyFire() {
    return shooterPIDSubsystem.getController().atSetpoint()
        && hoodPIDSubsystem.getController().atSetpoint();
        
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
