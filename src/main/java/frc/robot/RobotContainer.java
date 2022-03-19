// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.StickDrive;
import frc.robot.commands.TurretTrackTarget;
import frc.robot.commands.shooterPID;
import frc.robot.commands.FrontIntake.FrontIntakeEnable;
import frc.robot.commands.RearIntake.RearIntakeEnable;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final DriveSubsystem DriveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  private final PneumaticsSubsystem PneumaticsSubsystem = new PneumaticsSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final IndexerSubsystem IndexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final VisionSubsystem VisionSubsystem = new VisionSubsystem();
  private final XboxController driveController = new XboxController(0);
  private final XboxController operateController = new XboxController(1);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    DriveSubsystem.setDefaultCommand(

        new StickDrive(() -> driveController.getRawAxis(2) - driveController.getRawAxis(3),
            () -> -driveController.getRawAxis(0), DriveSubsystem));

  }

  private void configureButtonBindings() {
    // operate a button = front intake enable
    new JoystickButton(operateController, 1)
        .whileActiveOnce(new FrontIntakeEnable(IntakeSubsystem, PneumaticsSubsystem,
            IndexerSubsystem));

    // operate y button = rear intake enable
    new JoystickButton(operateController, 4)
        .whileActiveOnce(new RearIntakeEnable(IntakeSubsystem, PneumaticsSubsystem,
            IndexerSubsystem));

    new JoystickButton(driveController, 3).whileActiveOnce(new shooterPID(shooterSubsystem, () -> 3000));

    new JoystickButton(driveController, 1)
        .whileActiveOnce(new TurretTrackTarget(turretSubsystem, () -> VisionSubsystem.getTargetYaw()));
  }

}
