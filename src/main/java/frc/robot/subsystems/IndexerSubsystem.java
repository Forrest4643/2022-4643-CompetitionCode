package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IndexerConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerSubsystem extends SubsystemBase {
    
    public final CANSparkMax Front = new CANSparkMax(IndexerConstants.frontID, MotorType.kBrushless);
    public final CANSparkMax Rear = new CANSparkMax(IndexerConstants.rearID, MotorType.kBrushless);

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

    public void resetFrontEncoder() {
        fEncoder.setPosition(0);
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

    public void resetRearEncoder() {
        rEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("frontIndexerRPM", fEncoder.getVelocity());
        SmartDashboard.putNumber("rearIndexerRPM", rEncoder.getVelocity());
    }
}