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
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.SerialPort;

public class Sensors extends SubsystemBase {

  AHRS ahrs;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final I2C.Port mXpPorti2c = I2C.Port.kMXP;
  private final ColorSensorV3 m_frontSense = new ColorSensorV3(i2cPort);
  private final ColorSensorV3 m_rearSense = new ColorSensorV3(mXpPorti2c); 
  private final AnalogInput m_index1 = new AnalogInput(1);

  Color m_frontColor;

  Color m_rearColor;

 

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

    double frontProx = m_frontSense.getProximity();

    double rearProx = m_rearSense.getProximity();

    m_frontColor = m_frontSense.getColor();

    m_rearColor = m_rearSense.getColor();

    SmartDashboard.putNumber("frontProx", frontProx);

    SmartDashboard.putNumber("rearProx", rearProx);

    SmartDashboard.putNumber("frontRedValue", m_frontColor.red);
    SmartDashboard.putNumber("frontBlueValue", m_frontColor.blue);

    SmartDashboard.putNumber("rearRedValue", m_rearColor.red);
    SmartDashboard.putNumber("rearBlueValue", m_rearColor.blue);

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

  public boolean frontBall() {
    return(m_frontSense.getProximity() > IndexerConstants.frontThresh);
  }

  public boolean rearBall() {
    return(m_rearSense.getProximity() > IndexerConstants.rearThresh);
  }



 

  public boolean correctFrontCargo() {
    if (m_frontColor.blue >= IndexerConstants.blueThresh
        && DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      return true;
    } else if ((m_frontColor.red >= IndexerConstants.redThresh
        && DriverStation.getAlliance() == DriverStation.Alliance.Red)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean correctRearCargo() {
    if (m_rearColor.blue >= IndexerConstants.blueThresh
        && DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      return true;
    } else if ((m_rearColor.red >= IndexerConstants.redThresh
        && DriverStation.getAlliance() == DriverStation.Alliance.Red)) {
      return true;
    } else {
      return false;
    }
  }
}
