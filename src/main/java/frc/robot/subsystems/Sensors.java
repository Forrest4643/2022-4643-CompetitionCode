// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.SerialPort;

public class Sensors extends SubsystemBase {

  AHRS ahrs;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final I2C.Port mXpPorti2c = I2C.Port.kMXP;
  private final ColorSensorV3 m_topSense = new ColorSensorV3(i2cPort);
  private final ColorSensorV3 m_bottomSense = new ColorSensorV3(mXpPorti2c); 
  private final AnalogInput m_index1 = new AnalogInput(1);
  private final AnalogInput m_turret0 = new AnalogInput(0);

  Color m_topColor;

  Color m_bottomColor;

 

  /** Creates a new IndexSensors. */
  public Sensors() {
    //instantiate navx over USB
    try {
      ahrs = new AHRS(SerialPort.Port.kUSB);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-USB: " + ex.getMessage(), true);
    }

  }

  @Override
  public void periodic() {

    double topProx = m_topSense.getProximity();

    double bottomProx = m_bottomSense.getProximity();

    m_topColor = m_topSense.getColor();

    m_bottomColor = m_bottomSense.getColor();

    SmartDashboard.putNumber("topProx", topProx);

    SmartDashboard.putNumber("bottomProx", bottomProx);

    SmartDashboard.putNumber("topRedValue", m_topColor.red);
    SmartDashboard.putNumber("topBlueValue", m_topColor.blue);

    SmartDashboard.putNumber("bottomRedValue", m_bottomColor.red);
    SmartDashboard.putNumber("bottomBlueValue", m_bottomColor.blue);

    SmartDashboard.putNumber("Yaw", yaw());
    SmartDashboard.putNumber("Pitch", pitch());
    SmartDashboard.putNumber("Roll", roll());

    SmartDashboard.putBoolean("turretZero", turretZero());
    SmartDashboard.putNumber("turretZeroVolt", m_turret0.getVoltage());


  }

  public double yaw() {
    return ahrs.getYaw();
  }
  public double pitch() {
    return ahrs.getPitch();
  }
  public double roll() {
    return ahrs.getRoll();
  }

  public boolean index1() {
    return (m_index1.getVoltage() > IndexerConstants.thresh1);
  }

  public boolean topBall() {
    return(m_topSense.getProximity() > IndexerConstants.topThresh);
  }

  public boolean bottomBall() {
    return(m_bottomSense.getProximity() > IndexerConstants.bottomThresh);
  }

  public boolean turretZero() {
    return (m_turret0.getVoltage() < TurretConstants.zeroThresh);
  }



 

  public boolean topCargoBlue() {
    if (m_topColor.blue >= IndexerConstants.blueThresh) {
      return true;
    } else {
      return false;
    }
  }
  

  public boolean bottomCargoBlue() {
    if (m_bottomColor.blue >= IndexerConstants.blueThresh) {
      return true;
    } else {
      return false;
    }
  }
}
