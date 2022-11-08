// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class driveAim extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private boolean highGoal;

  /** Creates a new driveAim. */
  public driveAim(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, boolean highGoal) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.highGoal = highGoal;
    }

  private PIDController driveSteer = new PIDController(DriveConstants.steerkP, DriveConstants.steerkP,
      DriveConstants.steerkD);
  private PIDController driveMove = new PIDController(DriveConstants.drivekP, DriveConstants.drivekI,
      DriveConstants.drivekD);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSteer.setSetpoint(0);
    driveSteer.setTolerance(.1);
    driveMove.setTolerance(3);
    if (highGoal) {
      System.out.println("highGoal!");
      driveMove.setSetpoint(DriveConstants.highGoal);
    } else {
      System.out.println("lowGoal!");
      driveMove.setSetpoint(DriveConstants.lowGoal);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Speed = driveSteer.calculate(visionSubsystem.getTargetYaw());
    double turnRate = driveMove.calculate(visionSubsystem.getTargetDistanceIN());
    driveSubsystem.setDrive(Speed, turnRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
