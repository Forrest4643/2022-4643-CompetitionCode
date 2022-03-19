// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FrontIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexerWheelsOn;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FrontIntakeEnable extends ParallelCommandGroup {
  /** Creates a new IntakeOn. */
  public FrontIntakeEnable(IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem,
      IndexerSubsystem indexerSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FrontIntakeOn(intakeSubsystem),
        new FrontIntakeOpen(pneumaticsSubsystem),
        new indexerWheelsOn(indexerSubsystem));
        
  }
}
