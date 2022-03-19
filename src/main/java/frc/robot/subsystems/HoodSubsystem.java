// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {

  private final CANSparkMax hoodMotor = new CANSparkMax(HoodConstants.hoodID, MotorType.kBrushless);

  // ticks to inches conversion factor
  private final RelativeEncoder hoodEncoder = hoodMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,
      (int) (1 / 514718.5404));

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, (float) 3.7);
    hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) .25);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hoodPositionIN", getHoodPositionIN());
    SmartDashboard.putNumber("hoodVelcity", getHoodVelocity());

  }

  public double getHoodVelocity() {
    return hoodEncoder.getVelocity();
  }

  public double getHoodPositionIN() {
    return hoodEncoder.getPosition();
  }

  public void setHoodMotor(double speed) {
    hoodMotor.set(speed);
  }
}
