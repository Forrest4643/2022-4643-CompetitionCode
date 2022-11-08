// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.commands.TurretOLD;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.hoodPID;
import frc.robot.commands.shooterPID;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends CommandBase {
  private VisionSubsystem visionSubsystem;
  private TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private HoodSubsystem hoodSubsystem;
  private XboxController drivercontroller;
  private IndexerSubsystem indexerSubsystem;
  private hoodPID hoodPID;
  private shooterPID shooterPID;
  private TurretTrackTarget turretTrackTarget;

  /** Creates a new AutoAim. */
  public AutoAim(VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem, XboxController driverController, IndexerSubsystem indexerSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.drivercontroller = driverController;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turretSubsystem.turretPositionDEG() >= 130 || turretSubsystem.turretPositionDEG() <= -130) {
      alertDriver(turretSubsystem.turretPositionDEG());
    }

    double targetDist = visionSubsystem.getTargetDistanceIN();

    calcHoodPos(targetDist);

    if (hoodPID.getController().atSetpoint() && turretTrackTarget.getController().atSetpoint()) {

      calcShooterRPM(targetDist);

      if (shooterPID.getController().atSetpoint()) {

      }

    }

  }

  // alerts driver which direction to turn the robot for aiming
  private void alertDriver(double turretPosition) {
    if (turretPosition > 0) {
      drivercontroller.setRumble(RumbleType.kLeftRumble, 1);
    }
    if (turretPosition < 0) {
      drivercontroller.setRumble(RumbleType.kRightRumble, 1);
    }
  }

  private void calcHoodPos(double targetDist) {

    // calculates hood position based on Shooter Calculations spreadsheet
    double HoodPos = (HoodConstants.quadAimC + (HoodConstants.quadAimB * targetDist)
        + (Math.pow(HoodConstants.quadAimA * targetDist, 2)));
    hoodPID.getController().setSetpoint(HoodPos);

    SmartDashboard.putNumber("calculated hoodPos", HoodPos);
  }

  // calculates shooterRPM based on Shooter Calculations spreadsheet
  private void calcShooterRPM(double targetDist) {

    double shooterRPM = (ShooterConstants.quadAimC + ((ShooterConstants.quadAimB * targetDist)
        + (Math.pow((ShooterConstants.quadAimA * targetDist), 2))));

    shooterPID.getController().setSetpoint(shooterRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // disable PIDS
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
