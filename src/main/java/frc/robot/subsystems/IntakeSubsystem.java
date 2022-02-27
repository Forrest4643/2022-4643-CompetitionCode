package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    public final CANSparkMax Front = new CANSparkMax(IntakeConstants.frontID, MotorType.kBrushless);
  //  public final CANSparkMax Rear = new CANSparkMax(IntakeConstants.rearID, MotorType.kBrushless);

    private RelativeEncoder fEncoder = Front.getEncoder();
   // private RelativeEncoder rEncoder = Rear.getEncoder();

    public void setFrontWheels(Boolean on) {
        if (on = true) {
            Front.set(-1);
        } else {
            Front.set(0);
        }
    }

    public double getFrontIntakeSpeed() {
        return fEncoder.getVelocity();
    }

    // public void setRearWheels(Boolean on) {
    //     if (on = true) {
    //         Rear.set(-1);
    //     } else {
    //         Rear.set(0);
    //     }
    // }

    // public double getRearIntakeSpeed() {
    //     return rEncoder.getVelocity();
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("frontIntakeRPM", fEncoder.getVelocity());
      //  SmartDashboard.putNumber("rearIntakeRPM", rEncoder.getVelocity());
    }

}
