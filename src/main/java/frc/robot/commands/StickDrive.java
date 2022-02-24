package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.DriveSubsystem;

public class StickDrive extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_Speed;
  private final DoubleSupplier m_turnRate;
  private final BooleanSupplier m_quickTurn;

  public StickDrive(DoubleSupplier Speed, DoubleSupplier turnRate, BooleanSupplier quickTurn, DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_Speed = Speed;
    m_turnRate = turnRate;
    m_quickTurn = quickTurn;
    addRequirements(m_driveSubsystem);
  }

  @Override
  public void execute() {
    m_driveSubsystem.setDrive(m_Speed.getAsDouble(), m_turnRate.getAsDouble(), m_quickTurn.getAsBoolean());
  }

}
