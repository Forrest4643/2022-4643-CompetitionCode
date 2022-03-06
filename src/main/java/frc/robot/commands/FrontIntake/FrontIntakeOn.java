package frc.robot.commands.FrontIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class FrontIntakeOn extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    public FrontIntakeOn(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("FrontIntake On!");
    }

    @Override
    public void execute() {
        intakeSubsystem.setFrontWheels(true);

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FrontIntake Off!");
        intakeSubsystem.setFrontWheels(false);

    }

}
