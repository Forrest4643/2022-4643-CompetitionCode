// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Sensors;

public class TurretPIDSubsystem extends PIDSubsystem {

  private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretID, MotorType.kBrushless);
  private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
  private final Sensors m_sensors;
  
  private final VisionSubsystem m_visionsubsystem;
  /** Creates a new TurretPIDSubsystem. */
  public TurretPIDSubsystem(VisionSubsystem m_visionsubsystem, Sensors m_sensors) {
    super(
        // The PIDController used by the subsystem
        new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD));

        getController().setTolerance(TurretConstants.tolerance);

        turretEncoder.setPositionConversionFactor(TurretConstants.turretTicksToDegrees);

        this.m_visionsubsystem = m_visionsubsystem;
        this.m_sensors = m_sensors;
  }

  public void zeroTurret() {
    while (!m_sensors.turretZero()) {
      turretMotor.set(.1);
    }
      turretMotor.set(0);
    if (m_sensors.turretZero()) {
      turretEncoder.setPosition(0);
    }
  }

  @Override
  public void useOutput(double output, double setpoint) {
    double limitedOutput;
    if (turretPositionDEG() > TurretConstants.turretForwardLimit) {
      limitedOutput = MathUtil.clamp(output, -11, 0);
      //System.out.println("FWDLIMIT");
    } else if (turretPositionDEG() < TurretConstants.turretReverseLimit) {
      limitedOutput = MathUtil.clamp(output, 0, 11);
      //System.out.println("REVLIMIT");
    } else {
      limitedOutput = output;
    }
    turretMotor.set(limitedOutput);
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