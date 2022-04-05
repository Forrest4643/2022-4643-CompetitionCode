// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_cMotor = new CANSparkMax(ClimberConstants.climbID, MotorType.kBrushless);
  private final RelativeEncoder m_cEncoder = m_cMotor.getEncoder();
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_cMotor.setIdleMode(IdleMode.kBrake);
    m_cEncoder.setPositionConversionFactor(ClimberConstants.conversionFactor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor(double volts) {
    m_cMotor.setVoltage(volts);
  }

  public double climberInches() {
    //(degrees rotated / 360) * 2pi * spool radius
    return (m_cEncoder.getPosition() / 360) * 2.35619449;
  }
}
