package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerSubsystem extends SubsystemBase {
    
    public final CANSparkMax Front = new CANSparkMax(11, MotorType.kBrushless);
    public final CANSparkMax Rear = new CANSparkMax(12, MotorType.kBrushless);

    private RelativeEncoder fEncoder = Front.getEncoder();
    private RelativeEncoder rEncoder = Rear.getEncoder();
    
    public void setFrontIndexerSpeed(double Speed) {
        Front.set(Speed);
    }

    public double getFrontIndexerSpeed() {
        return fEncoder.getVelocity();
    }

    public double getFrontIndexerPosition() {
        return fEncoder.getPosition();
    }

    public void setRearIndexerSpeed(double Speed) {
        Rear.set(Speed);
    }

    public double getRearIndexerSpeed() {
        return rEncoder.getVelocity();
    }

    public double getRearIndexerPosition() {
        return rEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("frontIndexerRPM", fEncoder.getVelocity());
        SmartDashboard.putNumber("rearIndexerRPM", rEncoder.getVelocity());
    }
}
