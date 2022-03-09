// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RearIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class RearIntakeOpen extends CommandBase {
  private PneumaticsSubsystem pneumaticsSubsystem;

  public RearIntakeOpen(PneumaticsSubsystem pneumaticsSubsystem) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    addRequirements(pneumaticsSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("RearIntake Open!");
    pneumaticsSubsystem.rearIntakeOpen();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("RearIntake Closed!");
    pneumaticsSubsystem.rearIntakeClosed();

  }
}
