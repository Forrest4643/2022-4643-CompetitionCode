// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SetShooter extends CommandBase {
    private final ShooterPIDSubsystem shooterPIDSubsystem;
    private final VisionSubsystem visionSubsystem;

    public SetShooter(ShooterPIDSubsystem shooterPIDSubsystem, VisionSubsystem visionSubsystem) {
        this.shooterPIDSubsystem = shooterPIDSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(shooterPIDSubsystem, visionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("SetLauncher Started!");
        shooterPIDSubsystem.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterPIDSubsystem.setSetpoint(shooterRPM());
        SmartDashboard.putNumber("shooterRPM", shooterRPM());
    }

    private double shooterRPM() {
        return visionSubsystem.getTargetDistance() * 1;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("SetLauncher Ended!");
        shooterPIDSubsystem.disable();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
