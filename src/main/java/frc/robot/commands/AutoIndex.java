// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class AutoIndex extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private IndexerSubsystem m_indexerSubsystem;
  private PneumaticsSubsystem m_pneumaticsSubsystem;
  private BooleanSupplier m_forward; 
  private BooleanSupplier m_reverse; 
  /** Creates a new AutoIndex. */
  public AutoIndex(IntakeSubsystem m_intakeSubsystem, IndexerSubsystem m_indexerSubsystem,
      PneumaticsSubsystem m_pneumaticsSubsystem, BooleanSupplier m_forward, BooleanSupplier m_reverse) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_indexerSubsystem = m_indexerSubsystem;
    this.m_pneumaticsSubsystem = m_pneumaticsSubsystem;
    this.m_forward = m_forward;
    this.m_reverse = m_reverse;
    addRequirements(m_intakeSubsystem, m_indexerSubsystem);
  }

  BangBangController m_indexBangController = new BangBangController();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexerSubsystem.Front.setIdleMode(IdleMode.kBrake);
    m_indexerSubsystem.Rear.setIdleMode(IdleMode.kBrake);
    m_indexBangController.setTolerance(IndexerConstants.bangTolerance);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    manual();
   
  }

  private void manual() {
    if (m_pneumaticsSubsystem.rearStatus()) {
      m_intakeSubsystem.rearWheelsOn();
    } else {
      m_intakeSubsystem.rearWheelsOff();
    }

    if (m_pneumaticsSubsystem.frontStatus()) {
      m_intakeSubsystem.frontWheelsOn();
    } else {
      m_intakeSubsystem.frontWheelsOff();
    }

    if (m_forward.getAsBoolean()) {
      m_indexerSubsystem.wheelsOn();
    } else if (m_reverse.getAsBoolean()) {
      m_indexerSubsystem.wheelsReverse();
    } else {
      m_indexerSubsystem.wheelsOff();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.frontWheelsOff();
    m_intakeSubsystem.rearWheelsOff();
    m_indexerSubsystem.wheelsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
