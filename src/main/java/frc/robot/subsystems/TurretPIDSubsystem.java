// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TurretConstants;

public class TurretPIDSubsystem extends PIDSubsystem {

  private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretID, MotorType.kBrushless);
  private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
  
  /** Creates a new TurretPIDSubsystem. */
  public TurretPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD));

        getController().setTolerance(TurretConstants.tolerance);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    turretMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return turretPositionDEG();
  }

  public double turretPositionDEG() {
    return turretEncoder.getPosition() * TurretConstants.turretTicksToDegrees;
  }

}
