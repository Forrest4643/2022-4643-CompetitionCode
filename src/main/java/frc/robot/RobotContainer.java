// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.StickDrive;
import frc.robot.commands.FrontIntake.FrontIndexerOn;
import frc.robot.commands.FrontIntake.FrontIntakeEnable;
import frc.robot.commands.FrontIntake.FrontIntakeOn;
import frc.robot.commands.FrontIntake.FrontIntakeOpen;
import frc.robot.commands.RearIntake.RearIntakeEnable;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final DriveSubsystem DriveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  private final PneumaticsSubsystem PneumaticsSubsystem = new PneumaticsSubsystem();
  private final ShooterPIDSubsystem ShooterPIDSubsystem = new ShooterPIDSubsystem();
  private final IndexerSubsystem IndexerSubsystem = new IndexerSubsystem();
  // private final VisionSubsystem VisionSubsystem = new VisionSubsystem();

  private final XboxController driveController = new XboxController(0);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    DriveSubsystem.setDefaultCommand(
        new StickDrive(() -> driveController.getRawAxis(1), () -> -driveController.getRawAxis(4),
            () -> driveController.getRawButton(10), DriveSubsystem));
  }

  private void configureButtonBindings() {
    new JoystickButton(driveController, 1)
        .whileActiveOnce(new FrontIntakeEnable(IntakeSubsystem, PneumaticsSubsystem, IndexerSubsystem));
    new JoystickButton(driveController, 4)
        .whileActiveOnce(new RearIntakeEnable(IntakeSubsystem, PneumaticsSubsystem, IndexerSubsystem));
  }

}
