// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.HoodConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class HoodPIDSubsystem extends PIDSubsystem {
  /** Creates a new HoodPIDSubsystem. */
  private final CANSparkMax hoodMotor = new CANSparkMax(HoodConstants.hoodID, MotorType.kBrushless);
  private final RelativeEncoder hoodEncoder = hoodMotor.getAlternateEncoder(8192);

  private final ElevatorFeedforward hoodFeedForward = 
  new ElevatorFeedforward(HoodConstants.hoodkS, HoodConstants.hoodkG, HoodConstants.hoodkV);
  public HoodPIDSubsystem() {

    
    super(new PIDController(
        HoodConstants.hoodkP, HoodConstants.hoodkI, HoodConstants.hoodkD));
    getController().setTolerance(0.15);
    hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, HoodConstants.hoodReverseLimit);
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, HoodConstants.hoodForwardLimit);

  }

  @Override
  public void useOutput(double output, double setpoint) {
  hoodMotor.set(output + hoodFeedForward.calculate(HoodConstants.hoodInPerSec, HoodConstants.hoodAccInPerSec));  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return hoodEncoder.getPosition();
  }
}
