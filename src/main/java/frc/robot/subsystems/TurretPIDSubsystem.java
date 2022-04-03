// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TurretConstants;

public class TurretPIDSubsystem extends PIDSubsystem {

  private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretID, MotorType.kBrushless);
  private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
  
  private final VisionSubsystem m_visionsubsystem;
  /** Creates a new TurretPIDSubsystem. */
  public TurretPIDSubsystem(VisionSubsystem m_visionsubsystem) {
    super(
        // The PIDController used by the subsystem
        new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD));

        getController().setTolerance(TurretConstants.tolerance);

        turretEncoder.setPositionConversionFactor(TurretConstants.turretTicksToDegrees);

        turretMotor.setSoftLimit(SoftLimitDirection.kForward, TurretConstants.turretForwardLimit);
        turretMotor.setSoftLimit(SoftLimitDirection.kReverse, TurretConstants.turretReverseLimit);

        this.m_visionsubsystem = m_visionsubsystem;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    turretMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_visionsubsystem.getTargetYaw();
  }

  public double turretPositionDEG() {
    return turretEncoder.getPosition();
  }

}

//21645387