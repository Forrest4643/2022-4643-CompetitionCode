package frc.robot.subsystems;

import frc.robot.Constants.PNConstants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
    Compressor pcmCompressor = new Compressor(PNConstants.compressorID, PneumaticsModuleType.CTREPCM);

    DoubleSolenoid frontIntake = new DoubleSolenoid (PneumaticsModuleType.CTREPCM, PNConstants.frontForwardID, PNConstants.frontReverseID);
    DoubleSolenoid rearIntake = new DoubleSolenoid (PneumaticsModuleType.CTREPCM, PNConstants.rearForwardID, PNConstants.rearReverseID);
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pswitch Value", pcmCompressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("Compressor Enabled?", pcmCompressor.enabled());
        SmartDashboard.putNumber("Compressor Current", pcmCompressor.getCurrent());
    }
    
    public boolean getPswitch() {
        return pcmCompressor.getPressureSwitchValue();
    }

    public boolean getCompressorStatus(){
        return pcmCompressor.enabled();
    }

    public double getCompressorCurrent() {
        return pcmCompressor.getCurrent();
    }

    public void frontIntakePosition(Boolean Open) {
        if (Open = true){
            frontIntake.set(Value.kForward);
        }
        if (Open = false){
            frontIntake.set(Value.kReverse);
        }       
    }
    public void rearIntakePosition(Boolean Open) {
        if (Open = true){
            rearIntake.set(Value.kForward);
        }
        if (Open = false){
            rearIntake.set(Value.kReverse);
        }       
    }
}