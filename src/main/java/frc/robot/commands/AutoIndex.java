// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexSensors;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class AutoIndex extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private IndexerSubsystem m_indexerSubsystem;
  private PneumaticsSubsystem m_pneumaticsSubsystem;
  private IndexSensors m_indexsensors;
  private BooleanSupplier m_forward;
  private BooleanSupplier m_reverse;
  private IntSupplier m_POV;
  private boolean m_toggled;
  private boolean m_intakeOverride;

  /** Creates a new AutoIndex. */
  public AutoIndex(IntakeSubsystem m_intakeSubsystem, IndexerSubsystem m_indexerSubsystem,
      PneumaticsSubsystem m_pneumaticsSubsystem, IndexSensors m_indexsensors, BooleanSupplier m_forward,
      BooleanSupplier m_reverse,
      IntSupplier m_POV) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_indexerSubsystem = m_indexerSubsystem;
    this.m_pneumaticsSubsystem = m_pneumaticsSubsystem;
    this.m_indexsensors = m_indexsensors;
    this.m_forward = m_forward;
    this.m_reverse = m_reverse;
    this.m_POV = m_POV;
    addRequirements(m_intakeSubsystem, m_indexerSubsystem);
  }

  BangBangController m_indexBangController = new BangBangController();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexerSubsystem.Front.setIdleMode(IdleMode.kBrake);
    m_indexerSubsystem.Rear.setIdleMode(IdleMode.kBrake);
    m_indexBangController.setTolerance(IndexerConstants.bangTolerance);
    m_toggled = false;
    m_intakeOverride = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_POV.getAsInt() == 90) {
      m_toggled = true;
    }
    if (m_POV.getAsInt() == 270) {
      m_toggled = false;

    }

    if (m_toggled) {
      manual();
    } else {
      auto();
    }
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

  private void auto() {
    // sets intake direction to inwards if there is nothing at index 0
    if (!m_indexsensors.index0()) {
      m_intakeOverride = false;
    }

    // turns intake wheels on if the intake solonoid is open, and the override
    // condition is false.
    if (m_pneumaticsSubsystem.frontStatus() && !m_intakeOverride) {
      m_intakeSubsystem.frontWheelsOn();
    } else {
      m_intakeSubsystem.frontWheelsOff();
    }

    if (m_pneumaticsSubsystem.rearStatus() && !m_intakeOverride) {
      m_intakeSubsystem.rearWheelsOn();
    } else {
      m_intakeSubsystem.rearWheelsOff();
    }

    if (m_indexsensors.index0()) {
      // if 0
      if (m_indexsensors.index2()) {
        // if 2 and 0
        if (m_indexsensors.index1()) {
          // if 2, 1, and 0.

          // this override overrides the intakes automatically spinning inwards when the-
          // solenoid is open

          // this prevents us from carrying more than 2 balls.
          m_intakeOverride = true;
          m_intakeSubsystem.frontWheelsReverse();
        } else if (!m_indexsensors.index1()) {
          // if 2 and 0 but !1
          while (!m_indexsensors.index1()) {
            m_indexerSubsystem.wheelsReverse();
          }
          // turn off wheels after ball reaches 1
          m_indexerSubsystem.wheelsOff();
        }
      } else if (!m_indexsensors.index2()) {
        // if !2
        if (m_indexsensors.index1()) {
          // if !2 and 1
          while (!m_indexsensors.index2()) {
            m_indexerSubsystem.wheelsOn();
          }
          // turn off wheels after ball reaches 2
          m_indexerSubsystem.wheelsOff();
        } else if (!m_indexsensors.index1()) {
          // if !1 and !2 and 0, index in
          m_indexerSubsystem.wheelsOn();
        }
      }
    } else {
      // if theres no balls at 0, indexer wheels off.
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
