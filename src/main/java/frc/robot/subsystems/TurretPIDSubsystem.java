package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TurretConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretPIDSubsystem extends PIDSubsystem {

    private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretID, MotorType.kBrushless);

    private final RelativeEncoder turretEncoder = 
        turretMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 
            TurretConstants.turretTicksPerRev);
    private double m_turretPosition;

    public TurretPIDSubsystem() {
        super(new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD));
        getController().setTolerance(1);
        turretMotor.setSoftLimit(SoftLimitDirection.kForward, TurretConstants.turretForwardLimit);
        turretMotor.setSoftLimit(SoftLimitDirection.kReverse, TurretConstants.turretReverseLimit);
        turretMotor.setSmartCurrentLimit(10);

    }

    @Override
    public void periodic() {
        m_turretPosition = turretEncoder.getPosition() * TurretConstants.turretTicksToDegrees;
        SmartDashboard.putNumber("turret degrees", m_turretPosition);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        turretMotor.set(output);

    }

    public double turretPosition() {
        return m_turretPosition;
    }

    @Override
    protected double getMeasurement() {
        return m_turretPosition;
    }

}
