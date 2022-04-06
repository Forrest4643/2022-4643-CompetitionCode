// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class ActivateTurret extends CommandBase {
  private TrackTarget m_tracktarget;
  private LookForTarget m_lookfortarget;
  private VisionSubsystem m_visionsubsystem;
  /** Creates a new ActivateTurret. */
  public ActivateTurret(TrackTarget m_tracktarget, LookForTarget m_lookfortarget, VisionSubsystem m_visionsubsystem) {
    this.m_visionsubsystem = m_visionsubsystem;
    this.m_tracktarget = m_tracktarget;
    this.m_lookfortarget = m_lookfortarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_visionsubsystem.LEDon();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_visionsubsystem.hasTargets()) {
      if(m_lookfortarget.isScheduled()) {
        m_lookfortarget.cancel();
      }
      m_tracktarget.schedule(true);
    }
    if (!m_visionsubsystem.hasTargets()) {
      if (m_tracktarget.isScheduled()) {
      m_tracktarget.cancel();
      }
      m_lookfortarget.schedule(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_tracktarget.isScheduled()) {
      m_tracktarget.cancel();
    }

    if(m_lookfortarget.isScheduled()) {
      m_lookfortarget.cancel();
    }

    m_visionsubsystem.LEDoff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
