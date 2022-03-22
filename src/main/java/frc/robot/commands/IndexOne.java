// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexOne extends CommandBase {

  private IndexerSubsystem indexerSubsystem;
  /** Creates a new IndexOne. */
  public IndexOne(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(indexerSubsystem);
  }

  boolean m_primed;

  BangBangController indexBangController = new BangBangController();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexBangController.setTolerance(IndexerConstants.bangTolerance);
    indexBangController.setSetpoint(indexerSubsystem.getPosition() + IndexerConstants.oneBall);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexerSubsystem.setWheels(indexBangController.calculate(indexerSubsystem.getPosition()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexBangController.atSetpoint();
  }
}
