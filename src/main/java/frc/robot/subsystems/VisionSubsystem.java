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

    double m_targetDistanceMetersRAW;

    boolean m_hasTargets;

    public VisionSubsystem() {
        camera.setLED(VisionLEDMode.kOff);
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        SmartDashboard.putBoolean("result.hasTargets", result.hasTargets());

        m_hasTargets = result.hasTargets();

        if (result.hasTargets()) {
            m_targetYaw = result.getBestTarget().getYaw();
            m_targetDistanceMetersRAW = PhotonUtils.calculateDistanceToTargetMeters(
                    VisionConstants.cameraHeightMETERS,
                    VisionConstants.targetHeightMETERS,
                    VisionConstants.cameraAngleRAD,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
            SmartDashboard.putBoolean("hasTargets", result.hasTargets());
            SmartDashboard.putNumber("targetDistance", getTargetDistanceFT());

            m_targetDistanceMeters = VisionConstants.distC +
                    (VisionConstants.distB * m_targetDistanceMetersRAW)
                    + (Math.pow(m_targetDistanceMetersRAW, 2) * VisionConstants.distA);

        } else {
            m_targetYaw = 0;
            m_targetDistanceMeters = 0;
            SmartDashboard.putBoolean("hasTargets", result.hasTargets());
        }

    }

    public boolean hasTargets() {
        return m_hasTargets;
    }

    public double getTargetYaw() {
        return m_targetYaw;

    }

    public double getTargetDistanceFT() {
        return Units.metersToFeet(m_targetDistanceMetersRAW) + VisionConstants.distanceOffset;

    }

    public void LEDon() {

        camera.setLED(VisionLEDMode.kOn);
    }

    public void LEDoff() {
        camera.setLED(VisionLEDMode.kOff);

    }
}
