package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax Turret = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax Hood = new CANSparkMax(10, MotorType.kBrushless);

    //shooter encoders
    private RelativeEncoder lEncoder = leftMotor.getEncoder();
    private RelativeEncoder rEncoder = rightMotor.getEncoder();  

    private RelativeEncoder turretEncoder = Turret.getEncoder();

    private RelativeEncoder hoodEncoder = Hood.getEncoder();

    private final MotorControllerGroup Shooter = new MotorControllerGroup(leftMotor, rightMotor);

    
    public double getShooterSpeed() {
        //returns the avg of the two motor speeds
        return (lEncoder.getVelocity() + rEncoder.getVelocity()) / 2;
    }
    public void setShooterSpeed(double Speed) {
        Shooter.set(Speed);
    }

    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }
    public void setHoodSpeed(double Speed) {
        Hood.set(Speed);
    }

    public double getTurretPosition() {
        return turretEncoder.getPosition();
    }
    public void setTurretSpeed(double Speed) {
        Turret.set(Speed);
    }
}
