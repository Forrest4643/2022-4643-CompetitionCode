// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class IndexSensors extends SubsystemBase {

  private final AnalogInput indexFront = new AnalogInput(1);
  private final AnalogInput indexRear = new AnalogInput(2);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 towerColor = new ColorSensorV3(i2cPort);

  Color detectedColor;

  /** Creates a new IndexSensors. */
  public IndexSensors() {
  }

  @Override
  public void periodic() {

    double proximity = towerColor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

    detectedColor = towerColor.getColor();

    SmartDashboard.putNumber("redValue", detectedColor.red);
    SmartDashboard.putNumber("blueValue", detectedColor.blue);
    SmartDashboard.putBoolean("correctCargo", correctCargo());

  }

  public boolean indexFrontPres() {
    return (indexFront.getVoltage() < IndexerConstants.sensorThresh);
  }

  public boolean indexRearPres() {
    return (indexRear.getVoltage() < IndexerConstants.sensorThresh);
  }

  public boolean correctCargo() {
    if (indexFrontPres()) {
      if (detectedColor.blue >= IndexerConstants.blueThresh
          && DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        return true;
      } else if ((detectedColor.red >= IndexerConstants.redThresh
          && DriverStation.getAlliance() == DriverStation.Alliance.Red)) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }

  }
}
