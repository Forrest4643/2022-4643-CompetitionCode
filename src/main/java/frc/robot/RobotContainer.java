// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.StickDrive;
import frc.robot.commands.driveAim;
import frc.robot.commands.hoodPID;
import frc.robot.commands.shooterPID;
import frc.robot.commands.FrontIntake.FrontIntakeEnable;
import frc.robot.commands.Indexer.indexerWheelsOn;
import frc.robot.commands.Indexer.indexerWheelsReverse;
import frc.robot.commands.RearIntake.RearIntakeEnable;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexSensors;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final DriveSubsystem DriveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  private final PneumaticsSubsystem PneumaticsSubsystem = new PneumaticsSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final IndexSensors indexSensors = new IndexSensors();
  private final IndexerSubsystem IndexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  //private final VisionSubsystem VisionSubsystem = new VisionSubsystem();
  private final XboxController driveController = new XboxController(0);
  private final XboxController operateController = new XboxController(1);

  String trajectoryJSON = "C:/Users/arkap/OneDrive/Documents/FRC/2022/2022-Robot-Code/PathWeaver/output/Ball1.wpilib.json";
  private Trajectory Auto1 = new Trajectory();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Auto1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }

    DriveSubsystem.setDefaultCommand(

        new StickDrive(() -> driveController.getRawAxis(5) - driveController.getRawAxis(4),
            () -> driveController.getRawAxis(0), DriveSubsystem));


  }

  private void configureButtonBindings() {
  
   
  }     

  InstantCommand DriveSimStart = new InstantCommand(DriveSubsystem::DriveSiminit);

  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    
    // Reset odometry to the starting pose of the trajectory.
    DriveSubsystem.resetOdometry(Auto1.getInitialPose());

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            Auto1,
            DriveSubsystem::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            DriveSubsystem::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            DriveSubsystem::tankDriveVolts,
            DriveSubsystem);



    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> DriveSubsystem.tankDriveVolts(0, 0));
  }
}



