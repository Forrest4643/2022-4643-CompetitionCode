// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.commands.FrontIntake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class FrontIntakeOpen extends CommandBase {
  private PneumaticsSubsystem pneumaticsSubsystem;

  public FrontIntakeOpen(PneumaticsSubsystem pneumaticsSubsystem) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    addRequirements(pneumaticsSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("FrontIntake Open!");
    pneumaticsSubsystem.frontIntakeOpen();

  }  

  @Override
  public void end(boolean interrupted) {
      System.out.println("FrontIntake Closed!");
      pneumaticsSubsystem.frontIntakeClosed();

  }
}
