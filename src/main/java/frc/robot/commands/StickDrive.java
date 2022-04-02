package frc.robot.commands;

  import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class StickDrive extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final DoubleSupplier m_Speed;
  private final DoubleSupplier m_turnRate;
  private final BooleanSupplier m_autoAim;

  private PIDController driveSteer = new PIDController(DriveConstants.steerkP, DriveConstants.steerkP,
      DriveConstants.steerkD);

  public StickDrive(DoubleSupplier Speed, DoubleSupplier turnRate, BooleanSupplier autoAim,
      DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_driveSubsystem = driveSubsystem;
    m_Speed = Speed;
    m_turnRate = turnRate;
    m_autoAim = autoAim;

    driveSteer.setSetpoint(0);
    driveSteer.setTolerance(.1);

    addRequirements(m_driveSubsystem, m_visionSubsystem);
  }

  // This runs once per scheduler run
  @Override
  public void execute() {

    if (m_autoAim.getAsBoolean()) {
      double turnRate = driveSteer.calculate(m_visionSubsystem.getTargetYaw());
      m_driveSubsystem.setDrive(0, MathUtil.clamp(turnRate, -.5, .5));
      SmartDashboard.putNumber("driveSteer", turnRate);
    } else {
      m_driveSubsystem.setDrive(m_Speed.getAsDouble(), m_turnRate.getAsDouble());
    }
    // aiming drivetrain
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
