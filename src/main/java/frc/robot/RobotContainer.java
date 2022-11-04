// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    new JoystickButton(operateController, 4)
        .whileActiveOnce(new FrontIntakeEnable(IntakeSubsystem, PneumaticsSubsystem,
            IndexerSubsystem));

    // operate y button = rear intake enable
    new JoystickButton(operateController, 2)
        .whileActiveOnce(new RearIntakeEnable(IntakeSubsystem, PneumaticsSubsystem,
            IndexerSubsystem));

    // TESTING
    // new JoystickButton(operateController, 3).whenPressed(new
    // hoodPID(hoodSubsystem, () -> 1.25));
    // new JoystickButton(operateController, 1).whenPressed(new
    // hoodPID(hoodSubsystem, () -> 1.75));

    //x high goal aim
    new JoystickButton(driveController, 3).whileActiveOnce(new driveAim(DriveSubsystem, VisionSubsystem, true));
    new JoystickButton(driveController, 3).whileActiveOnce(new hoodPID(hoodSubsystem, () -> HoodConstants.highGoal));
    new JoystickButton(driveController, 3).whileActiveOnce(new shooterPID(shooterSubsystem, () -> ShooterConstants.highGoal));

    //b low goal aim
    new JoystickButton(driveController, 2).whileActiveOnce(new driveAim(DriveSubsystem, VisionSubsystem, false));
    new JoystickButton(driveController, 2).whileActiveOnce(new hoodPID(hoodSubsystem, () -> HoodConstants.lowGoal));
    new JoystickButton(driveController, 2).whileActiveOnce(new shooterPID(shooterSubsystem, () -> ShooterConstants.lowGoal));
    
    //l bump index rev
    new JoystickButton(operateController, 5).whileActiveOnce(new indexerWheelsReverse(IndexerSubsystem));

    //r bump index fwd
    new JoystickButton(operateController, 6).whileActiveOnce(new indexerWheelsOn(IndexerSubsystem));
   
  }     

  InstantCommand DriveSimStart = new InstantCommand(DriveSubsystem::DriveSiminit);


}
