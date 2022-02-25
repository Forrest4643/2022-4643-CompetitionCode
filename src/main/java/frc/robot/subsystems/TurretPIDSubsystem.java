package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TurretConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretPIDSubsystem extends PIDSubsystem {
private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretID, MotorType.kBrushless);

private RelativeEncoder turretEncoder = turretMotor.getEncoder();
    public TurretPIDSubsystem() {
        super(new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD));
        getController().setTolerance(0.5);
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
