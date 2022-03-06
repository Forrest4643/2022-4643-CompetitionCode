package frc.robot.commands.RearIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RearIntakeOn extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    public RearIntakeOn(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("RearIntake On!");
    }

    @Override
    public void execute() {
        intakeSubsystem.setRearWheels(true);

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RearIntake Off!");
        intakeSubsystem.setRearWheels(false);

    }

}



