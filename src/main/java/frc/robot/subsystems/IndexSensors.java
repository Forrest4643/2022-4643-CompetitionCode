// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class IndexSensors extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_towerColor = new ColorSensorV3(i2cPort);

  private final AnalogInput m_index0 = new AnalogInput(0);

  private final AnalogInput m_index1a = new AnalogInput(1);

  private final AnalogInput m_index1b = new AnalogInput(2);

  private final AnalogInput m_index2a = new AnalogInput(3);

  private final AnalogInput m_index2b = new AnalogInput(4);

  Color m_detectedColor;

  /** Creates a new IndexSensors. */
  public IndexSensors() {
  }

  @Override
  public void periodic() {

    double proximity = m_towerColor.getProximity();

    m_detectedColor = m_towerColor.getColor();

    SmartDashboard.putNumber("Proximity", proximity);

    

    SmartDashboard.putNumber("redValue", m_detectedColor.red);
    SmartDashboard.putNumber("blueValue", m_detectedColor.blue);
    SmartDashboard.putBoolean("correctCargo", correctCargo());

  }

  public boolean towerPres() {
    return (m_towerColor.getProximity() > IndexerConstants.colorProxThresh);
  }

  public boolean index0() {
    return (m_index0.getVoltage() >IndexerConstants.thresh0);
  }

  public boolean index1() {
    return (m_index1a.getVoltage() > IndexerConstants.thresh1a && m_index1b.getVoltage() > IndexerConstants.thresh1b);
  }

  public boolean index2() {
    return (m_index2a.getVoltage() > IndexerConstants.thresh2a && m_index2b.getVoltage() > IndexerConstants.thresh2b);
  }

 

  public boolean correctCargo() {
    if (m_detectedColor.blue >= IndexerConstants.blueThresh
        && DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      return true;
    } else if ((m_detectedColor.red >= IndexerConstants.redThresh
        && DriverStation.getAlliance() == DriverStation.Alliance.Red)) {
      return true;
    } else {
      return false;
    }
  }
}
