package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.DriveSubsystem;

public class StickDrive extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_Speed;
  private final DoubleSupplier m_turnRate;

  public StickDrive(DoubleSupplier Speed, DoubleSupplier turnRate,
      DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_Speed = Speed;
    m_turnRate = turnRate;
    addRequirements(m_driveSubsystem);
  }

  // This runs once per scheduler run
  @Override
  public void execute() {
    m_driveSubsystem.setDrive(m_Speed.getAsDouble(), m_turnRate.getAsDouble());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setDrive(0, 0);
  }

}
