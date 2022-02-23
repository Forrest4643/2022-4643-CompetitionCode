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

    public double getTargetYaw() {
        var result = camera.getLatestResult();
        return result.getBestTarget().getYaw();
    }

    public double getTargetDistance() {
        var result = camera.getLatestResult();
        return PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.cameraHeightMETERS,
            VisionConstants.targetHeightMETERS, 
            VisionConstants.cameraAngleRAD, 
            Units.degreesToRadians(result.getBestTarget().getPitch()));

    }

    public void setLED(boolean ON) {
        if(ON = true) {
            camera.setLED(VisionLEDMode.kOn);
        }

        if(ON = false) {
            camera.setLED(VisionLEDMode.kOff);
        }
    }
}
