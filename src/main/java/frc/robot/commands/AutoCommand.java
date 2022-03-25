// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommand extends ParallelCommandGroup {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private IndexerSubsystem indexerSubsystem;
  private HoodPIDSubsystem hoodPIDSubsystem;
  private ShooterPIDSubsystem shooterPIDSubsystem;
  /** Creates a new AutoCommand. */
  public AutoCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveDistance(driveSubsystem, DriveConstants.autoDist), new InstantCommand(indexerSubsystem::wheelsOn).alongWith(new AutoAim(driveSubsystem, visionSubsystem, shooterPIDSubsystem, hoodPIDSubsystem, indexerSubsystem)).alongWith(new WaitCommand(3)), new InstantCommand(indexerSubsystem::wheelsOff));
  }
}
