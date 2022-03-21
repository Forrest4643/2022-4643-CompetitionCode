// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexSensors;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;

public class AutoIndex extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private IndexerSubsystem indexerSubsystem;
  private PneumaticsSubsystem pneumaticsSubsystem;
 

  /** Creates a new AutoIndex. */
  public AutoIndex(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
      PneumaticsSubsystem pneumaticsSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    addRequirements(intakeSubsystem, indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pneumaticsSubsystem.rearStatus()) {
      intakeSubsystem.rearWheelsOn();
    } else {
      intakeSubsystem.rearWheelsOff();
    }

    if (pneumaticsSubsystem.frontStatus()){
      intakeSubsystem.frontWheelsOn();
    } else {
      intakeSubsystem.frontWheelsOff();
    }

    if (pneumaticsSubsystem.rearStatus() || pneumaticsSubsystem.frontStatus()) {
      indexerSubsystem.wheelsOn();
    } else {
      indexerSubsystem.wheelsOff();
    }
    
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.frontWheelsOff();
    intakeSubsystem.rearWheelsOff();
    indexerSubsystem.wheelsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
