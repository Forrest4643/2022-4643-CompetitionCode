// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

        private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
        private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
        private final Sensors m_sensors = new Sensors();
        private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
        private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
        private final ShooterPIDSubsystem m_shooterPIDsubsystem = new ShooterPIDSubsystem();
        private final HoodPIDSubsystem m_hoodPIDsubsystem = new HoodPIDSubsystem();
        private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
        private final ClimberSubsystem m_climbersubsystem = new ClimberSubsystem();
        private final TurretPIDSubsystem m_turretPIDsubsystem = new TurretPIDSubsystem(m_visionSubsystem, m_sensors);
        private final XboxController m_driveController = new XboxController(0);
        private final XboxController m_operateController = new XboxController(1);

        private final LookForTarget m_lookfortarget = new LookForTarget(m_turretPIDsubsystem);
        private final TrackTarget m_tracktarget = new TrackTarget(m_turretPIDsubsystem);
        private final TurretPosition m_turretposition = new TurretPosition(m_turretPIDsubsystem, -160);

        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                m_driveSubsystem.setDefaultCommand(
                                new StickDrive(m_driveSubsystem, m_driveController, m_turretPIDsubsystem));

        }

        private void configureButtonBindings() {

                new JoystickButton(m_driveController, 6).whenPressed(new InstantCommand(m_climbersubsystem::up))
                                .whenReleased(new InstantCommand(m_climbersubsystem::idle));

                new JoystickButton(m_driveController, 5).whenPressed(new InstantCommand(m_climbersubsystem::down))
                                .whenReleased(new InstantCommand(m_climbersubsystem::idle));

                new JoystickButton(m_operateController, 4)
                                .whenPressed(new InstantCommand(m_pneumaticsSubsystem::frontIntakeOpen))
                                .whenReleased(new InstantCommand(m_pneumaticsSubsystem::frontIntakeClosed));

                new JoystickButton(m_operateController, 2)
                                .whenPressed(new InstantCommand(m_pneumaticsSubsystem::rearIntakeOpen))
                                .whenReleased(new InstantCommand(m_pneumaticsSubsystem::rearIntakeClosed));

                new JoystickButton(m_operateController, 1).whileActiveOnce(
                                new AutoAim(m_hoodPIDsubsystem, m_visionSubsystem, m_shooterPIDsubsystem),
                                true);

                new JoystickButton(m_operateController, 1)
                                .whenPressed(new InstantCommand(m_pneumaticsSubsystem::compOff))
                                .whenReleased(new InstantCommand(m_pneumaticsSubsystem::compOn));

                new JoystickButton(m_operateController, 3)
                                .toggleWhenPressed(
                                                new ActivateTurret(m_tracktarget, m_lookfortarget, m_visionSubsystem,
                                                                m_turretPIDsubsystem),
                                                true);

                new JoystickButton(m_operateController, 12).toggleWhenPressed(
                                new HUB(m_turretposition, m_shooterPIDsubsystem, m_hoodPIDsubsystem,
                                                m_turretPIDsubsystem),
                                true);

                new JoystickButton(m_operateController, 11)
                                .whenPressed(new InstantCommand(m_shooterPIDsubsystem::backDrive))
                                .whenReleased(new InstantCommand(m_shooterPIDsubsystem::idleShooter));

                // ****FOR TESTING****

                // new JoystickButton(m_driveController, 3)
                // .whileHeld(new ParallelCommandGroup(new
                // InstantCommand(m_hoodPIDsubsystem::hoodClosed),
                // new InstantCommand(m_hoodPIDsubsystem::enable)))
                // .whenReleased(new InstantCommand(m_hoodPIDsubsystem::disable));
                // new JoystickButton(m_driveController, 2)
                // .whileHeld(new ParallelCommandGroup(new
                // InstantCommand(m_hoodPIDsubsystem::hoodOpen),
                // new InstantCommand(m_hoodPIDsubsystem::enable)))
                // .whenReleased(new InstantCommand(m_hoodPIDsubsystem::disable));
                
                new JoystickButton(m_driveController, 1)
                                .whenPressed(new InstantCommand(m_hoodPIDsubsystem::positionUpdate)
                                                .alongWith(new InstantCommand(m_turretPIDsubsystem::updatePosition)));
                new JoystickButton(m_driveController, 4)
                                .whenPressed(new InstantCommand(m_hoodPIDsubsystem::zeroHood));

        }

        public Command getAutonomousCommand() {

                // SmartDashboard.putBoolean("autonStart", true);
                // return new DriveDistance(m_driveSubsystem, DriveConstants.autoDist);

                // return new SequentialCommandGroup(
                // new ParallelDeadlineGroup(
                // new WaitCommand(3),
                // new HUB(m_turretposition, m_shooterPIDsubsystem, m_hoodPIDsubsystem,
                // m_turretPIDsubsystem),
                // new SequentialCommandGroup(
                // new WaitCommand(1.75),
                // new InstantCommand(m_indexerSubsystem::wheelsOn),
                // new WaitCommand(1),
                // new InstantCommand(m_indexerSubsystem::wheelsOff)
                // )
                // ),
                // new DriveDistance(m_driveSubsystem, DriveConstants.autoDist)
                // );

                return null;

        }

        public void teleInit() {
                m_intakeSubsystem.setDefaultCommand(
                                new AutoIndex(m_intakeSubsystem, m_indexerSubsystem, m_pneumaticsSubsystem, m_sensors,
                                                m_operateController));
        }

}
