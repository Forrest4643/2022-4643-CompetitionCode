package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class FrontIntakeSet extends InstantCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final Boolean on;

    public FrontIntakeSet(IntakeSubsystem intakeSubsystem, Boolean on) {
        this.on = on;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("FrontIntakeSet Started!");
        intakeSubsystem.setFrontWheels(on);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FrontIntakeSet Ended!");
    }


}
