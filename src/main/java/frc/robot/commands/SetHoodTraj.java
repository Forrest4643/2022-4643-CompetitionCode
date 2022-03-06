// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.HoodConstants;
// import frc.robot.subsystems.HoodPIDSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// public class SetHoodTraj extends CommandBase {
//   private final HoodPIDSubsystem hoodPIDSubsystem;
//   private final VisionSubsystem visionSubsystem;

//   public SetHoodTraj(HoodPIDSubsystem hoodPIDSubsystem, VisionSubsystem visionSubsystem) {
//     this.hoodPIDSubsystem = hoodPIDSubsystem;
//     this.visionSubsystem = visionSubsystem;
//     addRequirements(hoodPIDSubsystem, visionSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     System.out.println("SetHoodTraj Started!");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     hoodPIDSubsystem.setSetpoint(hoodSetpoint());
//   }

//   private double hoodSetpoint() {
//     return visionSubsystem.getTargetDistance() * HoodConstants.targetDistToSetpoint;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("SetHoodTraj Ended!");
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
