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
  private IntakeSubsystem intakeSubsystem;
  private IndexerSubsystem indexerSubsystem;
  private PneumaticsSubsystem pneumaticsSubsystem;
  private BooleanSupplier forward; 
  private BooleanSupplier reverse; 
  /** Creates a new AutoIndex. */
  public AutoIndex(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
      PneumaticsSubsystem pneumaticsSubsystem, BooleanSupplier forward, BooleanSupplier reverse) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    this.forward = forward;
    this.reverse = reverse;
    addRequirements(intakeSubsystem, indexerSubsystem);
  }

  BangBangController indexBangController = new BangBangController();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexerSubsystem.Front.setIdleMode(IdleMode.kBrake);
    indexerSubsystem.Rear.setIdleMode(IdleMode.kBrake);
    indexBangController.setTolerance(IndexerConstants.bangTolerance);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (pneumaticsSubsystem.rearStatus()) {
      intakeSubsystem.rearWheelsOn();
    } else {
      intakeSubsystem.rearWheelsOff();
    }

    if (pneumaticsSubsystem.frontStatus()) {
      intakeSubsystem.frontWheelsOn();
    } else {
      intakeSubsystem.frontWheelsOff();
    }

    /* if (pneumaticsSubsystem.rearStatus() || pneumaticsSubsystem.frontStatus()) {
      indexerSubsystem.wheelsOn();
    } else */ if (forward.getAsBoolean()) {
      indexerSubsystem.wheelsOn();
    } else if (reverse.getAsBoolean()) {
      indexerSubsystem.wheelsReverse();
    } else {
      indexerSubsystem.wheelsOff();
    }

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.frontWheelsOff();
    //intakeSubsystem.rearWheelsOff();
    indexerSubsystem.wheelsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
