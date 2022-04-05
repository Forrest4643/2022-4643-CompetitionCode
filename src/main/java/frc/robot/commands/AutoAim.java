// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends CommandBase {
  private VisionSubsystem m_visionSubsystem;
  private ShooterPIDSubsystem m_shooterPIDSubsystem;
  private HoodPIDSubsystem m_hoodPIDSubsystem;
  private TurretPIDSubsystem m_turretSubsystem;

  private PIDController driveSteer = new PIDController(DriveConstants.drivekP, DriveConstants.drivekI,
      DriveConstants.drivekD);

  /** Creates a new driveAim. */
  public AutoAim(HoodPIDSubsystem m_hoodPIDSubsystem, VisionSubsystem m_visionSubsystem,
      ShooterPIDSubsystem m_shooterPIDSubsystem, TurretPIDSubsystem m_turretSubsystem) {
    this.m_visionSubsystem = m_visionSubsystem;
    this.m_shooterPIDSubsystem = m_shooterPIDSubsystem;
    this.m_hoodPIDSubsystem = m_hoodPIDSubsystem;
    this.m_turretSubsystem = m_turretSubsystem;

    addRequirements(m_hoodPIDSubsystem, m_shooterPIDSubsystem, m_turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("AutoAim Started!");

    m_shooterPIDSubsystem.enable();
    m_hoodPIDSubsystem.enable();
    m_turretSubsystem.enable();
    m_visionSubsystem.LEDon();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_visionSubsystem.hasTargets()) {
      aim();
    } else {
      lookForTargets();
    }

  }

  private void lookForTargets() {

  }

  private void aim() {
    double targetDistance = m_visionSubsystem.getTargetDistanceFT();

    // aiming hood
    m_hoodPIDSubsystem.setSetpoint(MathUtil.clamp(
        HoodConstants.quadAimC +
            (HoodConstants.quadAimB * targetDistance) +
            (Math.pow(targetDistance, 2) * HoodConstants.quadAimA),

        65, 80));

    SmartDashboard.putNumber("hoodPositionDEG", m_hoodPIDSubsystem.getHoodPositionDEG());
    SmartDashboard.putNumber("hoodSetpoint", m_hoodPIDSubsystem.getSetpoint());

    // setting shooter RPM

    m_shooterPIDSubsystem.setSetpoint(ShooterConstants.quadAimD +
        (ShooterConstants.quadAimC * targetDistance) +
        (Math.pow((targetDistance), 2) * ShooterConstants.quadAimB) +
        (Math.pow(targetDistance, 3) * ShooterConstants.quadAimA));

    m_turretSubsystem.setSetpoint(0);
    SmartDashboard.putNumber("leftAmps", m_shooterPIDSubsystem.leftAmps());
    SmartDashboard.putNumber("rightAmps", m_shooterPIDSubsystem.rightAmps());

    SmartDashboard.putNumber("shooterRPM", m_shooterPIDSubsystem.getShooterRPM());
    SmartDashboard.putNumber("turretPositionDEG", m_turretSubsystem.turretPositionDEG());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterPIDSubsystem.disable();
    m_hoodPIDSubsystem.disable();
    m_visionSubsystem.LEDoff();
    m_turretSubsystem.disable();

    System.out.println("AutoAim Ended!");
  }

  public boolean readyFire() {
    return m_shooterPIDSubsystem.getController().atSetpoint()
        && m_hoodPIDSubsystem.getController().atSetpoint() && driveSteer.atSetpoint();

  }
}
