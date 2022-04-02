// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final Sensors m_sensors = new Sensors();
  private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final ShooterPIDSubsystem m_shooterPIDSubsystem = new ShooterPIDSubsystem();
  private final HoodPIDSubsystem m_hoodPIDSubsystem = new HoodPIDSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final TurretPIDSubsystem m_turretSubsystem = new TurretPIDSubsystem();
  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_operateController = new XboxController(1);
  

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveSubsystem.setDefaultCommand(new StickDrive(() -> m_driveController.getRawAxis(2) - m_driveController.getRawAxis(3),
        () -> -m_driveController.getRawAxis(0), m_driveSubsystem));

    m_intakeSubsystem.setDefaultCommand(new AutoIndex(m_intakeSubsystem, m_indexerSubsystem, m_pneumaticsSubsystem, m_sensors,
        () -> m_operateController.getRightBumper(), () -> m_operateController.getLeftBumper(), () -> m_operateController.getPOV()));

  }

  private void configureButtonBindings() {

    new JoystickButton(m_operateController, 4).whenPressed(new InstantCommand(m_pneumaticsSubsystem::frontIntakeOpen))
        .whenReleased(new InstantCommand(m_pneumaticsSubsystem::frontIntakeClosed));

    new JoystickButton(m_operateController, 2).whenPressed(new InstantCommand(m_pneumaticsSubsystem::rearIntakeOpen))
        .whenReleased(new InstantCommand(m_pneumaticsSubsystem::rearIntakeClosed));

    new JoystickButton(m_driveController, 1).whileActiveOnce(
        new AutoAim(m_hoodPIDSubsystem, m_visionSubsystem, m_shooterPIDSubsystem, m_driveSubsystem, m_turretSubsystem));
  }

  public Command getAutonomousCommand() {

    SmartDashboard.putBoolean("autonStart", true);
    return new AutoCommand(m_driveSubsystem, m_hoodPIDSubsystem, m_visionSubsystem, m_indexerSubsystem, m_shooterPIDSubsystem, m_sensors, m_intakeSubsystem, m_pneumaticsSubsystem, m_turretSubsystem);
  }

}
