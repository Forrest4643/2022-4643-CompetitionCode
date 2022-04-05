// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.HoodConstants;

public class HoodPIDSubsystem extends PIDSubsystem {

  private final CANSparkMax hoodMotor = new CANSparkMax(HoodConstants.hoodID, MotorType.kBrushless);

  // ticks to inches conversion factor
  private final RelativeEncoder hoodEncoder = hoodMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,
      8192);

  /** Creates a new HoodSubsystem. */
  public HoodPIDSubsystem() {
    super(new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD));

    getController().setTolerance(HoodConstants.PIDtolerance);

    hoodMotor.setInverted(true);
    hoodEncoder.setPositionConversionFactor(HoodConstants.conversionFactor);

  }


  public double getHoodVelocity() {
    return hoodEncoder.getVelocity();
  }

  public double getHoodPositionDEG() {
    return (-hoodEncoder.getPosition() * 5) + 65;
  }

  public double getHoodPositionIN() {
    return hoodEncoder.getPosition();
  }

  public void setHoodMotor(double speed) {
    hoodMotor.set(speed);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    double limitedOutput;
    if (getHoodPositionIN() < HoodConstants.ForwardLimit) {
      limitedOutput = MathUtil.clamp(output, -1, 0);
      System.out.println("FWDLIMIT");
    } else if (getHoodPositionIN() > HoodConstants.ReverseLimit) {
      limitedOutput = MathUtil.clamp(output, 0, 1);
      System.out.println("REVLIMIT");
    } else {
      limitedOutput = output;
    }
   hoodMotor.set(MathUtil.clamp(limitedOutput, -.5, .5));

  }

  @Override
  protected double getMeasurement() {
    return getHoodPositionDEG();
  }
}
