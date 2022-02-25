package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TurretConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretPIDSubsystem extends PIDSubsystem {

    private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretID, MotorType.kBrushless);

    private static final SparkMaxAlternateEncoder.Type turretEncoder = SparkMaxAlternateEncoder.Type.kQuadrature;

    private RelativeEncoder m_turretEncoder = turretMotor.getAlternateEncoder(4096);
    private double m_turretPosition;

    public TurretPIDSubsystem() {


        super(new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD));
        getController().setTolerance(0.15);
    }

    @Override
    public void periodic() {
        m_turretPosition = m_turretEncoder.getPosition() * TurretConstants.turretTicksToDegrees;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub

    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }

}
