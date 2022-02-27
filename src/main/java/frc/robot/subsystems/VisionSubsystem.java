package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import edu.wpi.first.math.util.Units;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera("photonvision");

    double m_targetYaw, m_targetDistanceMeters;

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        m_targetYaw = result.getBestTarget().getYaw();
        m_targetDistanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.cameraHeightMETERS,
                VisionConstants.targetHeightMETERS,
                VisionConstants.cameraAngleRAD,
                Units.degreesToRadians(result.getBestTarget().getPitch()));

        SmartDashboard.putNumber("targetYaw", m_targetYaw);
        SmartDashboard.putNumber("targetDistanceMETERS", m_targetDistanceMeters);
    }

    public double getTargetYaw() {
        return m_targetYaw;

    }

    public double getTargetDistance() {
        return m_targetDistanceMeters;

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
