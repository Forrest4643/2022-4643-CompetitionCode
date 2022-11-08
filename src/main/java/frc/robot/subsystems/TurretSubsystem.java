// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots


package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretID, MotorType.kBrushless);
  private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
  
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {}

  public double turretPositionDEG(){
    return turretEncoder.getPosition() * TurretConstants.turretTicksToDegrees;
  } 
  @Override
  public void periodic() {
  SmartDashboard.putNumber("turretPositionDEG", turretPositionDEG());
  }

  public void setTurretSpeed(double speed) {
    turretMotor.set(speed);
  }
}
