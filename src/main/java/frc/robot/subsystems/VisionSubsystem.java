package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera("photonvision");

    double m_targetYaw;

    double m_targetDistanceMeters;

    
   

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        
        SmartDashboard.putBoolean("result.hasTargets", result.hasTargets());

        if (result.hasTargets()) {
            m_targetYaw = result.getBestTarget().getYaw();
            m_targetDistanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
                    VisionConstants.cameraHeightMETERS,
                    VisionConstants.targetHeightMETERS,
                    VisionConstants.cameraAngleRAD,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
            SmartDashboard.putBoolean("hasTargets", result.hasTargets());
            SmartDashboard.putNumber("targetDistance", getTargetDistanceFT());

        } else {
            m_targetYaw = 0;
            m_targetDistanceMeters = 0;
            SmartDashboard.putBoolean("hasTargets", result.hasTargets());
        }

    }

    public double getTargetYaw() {
        return m_targetYaw;

    }

    public double getTargetDistanceFT() {
        return Units.metersToFeet(m_targetDistanceMeters) + VisionConstants.distanceOffset;

    }

    public void setLED(boolean ON) {
        if (ON = true) {
            camera.setLED(VisionLEDMode.kOn);
        }

        if (ON = false) {
            camera.setLED(VisionLEDMode.kOff);
        }
    }
}
