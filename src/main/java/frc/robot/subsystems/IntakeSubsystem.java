package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    public final CANSparkMax Front = new CANSparkMax(5, MotorType.kBrushless);
    public final CANSparkMax Rear = new CANSparkMax(6, MotorType.kBrushless);

    private RelativeEncoder fEncoder = Front.getEncoder(); 
    private RelativeEncoder rEncoder = Rear.getEncoder(); 

    public void setFrontIntakeSpeed(double Speed) {
        Front.set(Speed);
    }

    public double getFrontIntakeSpeed() {
        return fEncoder.getVelocity();
    }

    public void setRearIntakeSpeed(double Speed) {
        Rear.set(Speed);
    }

    public double getRearIntakeSpeed() {
        return rEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("frontIntakeRPM", fEncoder.getVelocity());
        SmartDashboard.putNumber("rearIntakeRPM", rEncoder.getVelocity());
    }

}
