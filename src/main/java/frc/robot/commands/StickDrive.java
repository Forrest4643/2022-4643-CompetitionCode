package frc.robot.commands;

  import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.DriveSubsystem;

public class StickDrive extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_Speed;
  private final DoubleSupplier m_turnRate;

  public StickDrive(DoubleSupplier Speed, DoubleSupplier turnRate,
      DriveSubsystem m_driveSubsystem) {

    this.m_driveSubsystem = m_driveSubsystem;
    m_Speed = Speed;
    m_turnRate = turnRate;

    addRequirements(m_driveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("StickDrive Start!");

  }

  // This runs once per scheduler run
  @Override
  public void execute() {

    m_driveSubsystem.setDrive(m_Speed.getAsDouble(), m_turnRate.getAsDouble());

  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setDrive(0, 0);
    System.out.println("StickDrive Ended!");
  }

}
