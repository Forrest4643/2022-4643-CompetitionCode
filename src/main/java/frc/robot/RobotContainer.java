// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.FrontIntakeSet;
import frc.robot.commands.StickDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem DriveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  private final PneumaticsSubsystem PneumaticsSubsystem = new PneumaticsSubsystem();
  private final ShooterPIDSubsystem ShooterPIDSubsystem = new ShooterPIDSubsystem();
  private final IndexerSubsystem IndexerSubsystem = new IndexerSubsystem();
  private final VisionSubsystem VisionSubsystem = new VisionSubsystem();

  private final XboxController driveController = new XboxController(1);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    DriveSubsystem.setDefaultCommand(
        new StickDrive(() -> driveController.getRawAxis(1), () -> driveController.getRawAxis(4),
            () -> driveController.getRawButtonPressed(6), DriveSubsystem));
  }

  private void configureButtonBindings() {
    //a button = intake on
    new JoystickButton(driveController, 1).whenPressed(new FrontIntakeSet(IntakeSubsystem, true));
    //b button = intake off
    new JoystickButton(driveController, 2).whenPressed(new FrontIntakeSet(IntakeSubsystem, false));
  }

  
}
