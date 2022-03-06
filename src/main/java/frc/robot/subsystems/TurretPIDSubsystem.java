// package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import frc.robot.Constants.TurretConstants;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMax.SoftLimitDirection;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// public class TurretPIDSubsystem extends PIDSubsystem {

//     private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretID, MotorType.kBrushless);

//     private RelativeEncoder turretEncoder = turretMotor.getAlternateEncoder(TurretConstants.turretTicksPerRev);
//     private double m_turretPosition;

//     public TurretPIDSubsystem() {
//         super(new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD));
//         getController().setTolerance(0.15);
//         turretMotor.setSoftLimit(SoftLimitDirection.kForward, TurretConstants.turretForwardLimit);
//         turretMotor.setSoftLimit(SoftLimitDirection.kReverse, TurretConstants.turretReverseLimit);

//     }

//     @Override
//     public void periodic() {
//         m_turretPosition = turretEncoder.getPosition() * TurretConstants.turretTicksToDegrees;
//     }

//     @Override
//     protected void useOutput(double output, double setpoint) {
//         turretMotor.set(output);

//     }

//     public double turretPosition() {
//         return m_turretPosition;
//     }

//     @Override
//     protected double getMeasurement() {
//         return m_turretPosition;
//     }

// }
