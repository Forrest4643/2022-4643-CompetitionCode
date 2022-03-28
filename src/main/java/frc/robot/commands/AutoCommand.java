// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommand extends SequentialCommandGroup {


  /** Creates a new AutoCommand. */
  public AutoCommand(DriveSubsystem m_driveSubsystem, HoodPIDSubsystem m_hoodPIDSubsystem, VisionSubsystem m_visionSubsystem, IndexerSubsystem m_indexerSubsystem,
    ShooterPIDSubsystem m_shooterPIDSubsystem, IntakeSubsystem m_intakeSubsystem, PneumaticsSubsystem m_pneumaticsSubsystem) {
    addCommands(
        new DriveDistance(m_driveSubsystem, DriveConstants.autoDist)
            .alongWith(new InstantCommand(m_pneumaticsSubsystem::rearIntakeOpen)),
        new AutoAim(m_hoodPIDSubsystem, m_visionSubsystem, m_shooterPIDSubsystem, m_driveSubsystem, m_indexerSubsystem)
            .raceWith(new WaitCommand(2)),
        new AutoIndex(m_intakeSubsystem, m_indexerSubsystem, m_pneumaticsSubsystem, () -> true, () -> false).alongWith(
            new AutoAim(m_hoodPIDSubsystem, m_visionSubsystem, m_shooterPIDSubsystem, m_driveSubsystem, m_indexerSubsystem))
            .raceWith(new WaitCommand(5)), new InstantCommand(m_pneumaticsSubsystem::rearIntakeClosed));
  }
}
