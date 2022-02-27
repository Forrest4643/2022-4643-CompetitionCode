package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ShooterPIDSubsystem extends PIDSubsystem {
    private final CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.leftMotorID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.rightMotorID, MotorType.kBrushless);

    // shooter encoders
    private RelativeEncoder lEncoder = leftMotor.getEncoder();
    private RelativeEncoder rEncoder = rightMotor.getEncoder();

    private double m_shooterSpeed;

    private final MotorControllerGroup m_Shooter = new MotorControllerGroup(leftMotor, rightMotor);

    public ShooterPIDSubsystem() {
        super(new PIDController(ShooterConstants.shooterkP, ShooterConstants.shooterkI, ShooterConstants.shooterkD));
        getController().setTolerance(50);
    }

    @Override
    public void periodic() {
        m_shooterSpeed = (lEncoder.getVelocity() + rEncoder.getVelocity()) / 2;
    }

    @Override
    public double getMeasurement() {
        return m_shooterSpeed;
    }

    @Override
    public void useOutput(double output, double setpoint) {
        m_Shooter.set(output);
    }

    public double getShooterSpeed() {
        // returns the avg of the two motor speeds
        return (lEncoder.getVelocity() + rEncoder.getVelocity()) / 2;
    }
}
