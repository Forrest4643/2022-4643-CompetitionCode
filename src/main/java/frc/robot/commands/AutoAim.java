// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private ShooterPIDSubsystem shooterPIDSubsystem;
  private HoodPIDSubsystem hoodPIDSubsystem;

  /** Creates a new driveAim. */
  public AutoAim(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      ShooterPIDSubsystem shooterPIDSubsystem, HoodPIDSubsystem hoodPIDSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.shooterPIDSubsystem = shooterPIDSubsystem;
    this.hoodPIDSubsystem = hoodPIDSubsystem;

  }

  private PIDController driveSteer = new PIDController(DriveConstants.steerkP, DriveConstants.steerkP,
      DriveConstants.steerkD);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSteer.setSetpoint(0);
    driveSteer.setTolerance(.1);
    System.out.println("AutoAim Started!");
    addRequirements(driveSubsystem, hoodPIDSubsystem, shooterPIDSubsystem);
    shooterPIDSubsystem.enable();
    hoodPIDSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetDistance = visionSubsystem.getTargetDistanceIN();
    double targetYaw = visionSubsystem.getTargetYaw();

    // aiming hood
    hoodPIDSubsystem.getController().setSetpoint(MathUtil.clamp(
        HoodConstants.quadAimC +
            (HoodConstants.quadAimB * targetDistance) +
            (Math.pow((HoodConstants.quadAimA * targetDistance), 2)),

        65, 80));

    // setting shooter RPM
    shooterPIDSubsystem.getController().setSetpoint(
        ShooterConstants.quadAimD +
            (ShooterConstants.quadAimC * targetDistance) +
            (Math.pow((ShooterConstants.quadAimB * targetDistance), 2) +
                (Math.pow((ShooterConstants.quadAimA * targetDistance), 3))));

    // aiming drivetrain
    double turnRate = driveSteer.calculate(targetYaw);
    driveSubsystem.setDrive(0, turnRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterPIDSubsystem.disable();
    hoodPIDSubsystem.disable();
    driveSubsystem.setDrive(0, 0);
    System.out.println("AutoAim Ended!");
  }

  public boolean readyFire() {
    return shooterPIDSubsystem.getController().atSetpoint()
        && hoodPIDSubsystem.getController().atSetpoint()
        && driveSteer.atSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
