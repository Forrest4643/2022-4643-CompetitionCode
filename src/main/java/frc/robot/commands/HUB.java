// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class HUB extends CommandBase {
  private ShooterPIDSubsystem m_shooterPIDsubsystem;
  private HoodPIDSubsystem m_hoodPIDsubsystem;
  private TurretPIDSubsystem m_turretPIDsubsystem;
  private TurretPosition m_turretposition = new TurretPosition(m_turretPIDsubsystem, -160);
  
  /** Creates a new HUB. */
  public HUB(TurretPosition m_turretposition, ShooterPIDSubsystem m_shooterPIDsubsystem, HoodPIDSubsystem m_hoodPIDsubsystem, TurretPIDSubsystem m_turretPIDsubsystem) {
    this.m_turretposition = m_turretposition;
    this.m_shooterPIDsubsystem = m_shooterPIDsubsystem;
    this.m_hoodPIDsubsystem = m_hoodPIDsubsystem;
    this.m_turretPIDsubsystem = m_turretPIDsubsystem;
    addRequirements(m_shooterPIDsubsystem, m_hoodPIDsubsystem, m_turretPIDsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hoodPIDsubsystem.enable();
    m_shooterPIDsubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPIDsubsystem.setSetpoint(2200);
    m_hoodPIDsubsystem.setSetpoint(65);
    if (!m_turretposition.isScheduled()) {
      m_turretposition.schedule();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterPIDsubsystem.disable();
    m_hoodPIDsubsystem.disable();
    if(m_turretposition.isScheduled()){
    m_turretposition.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
