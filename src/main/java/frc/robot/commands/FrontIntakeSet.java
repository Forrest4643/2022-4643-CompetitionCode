package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class FrontIntakeSet extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final boolean on;

    @Override
    public void initialize() {
        System.out.println("FrontIntakeSet Started!");
    }

    public FrontIntakeSet(IntakeSubsystem intakeSubsystem, boolean on) {
        this.on = on;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setFrontWheels(on);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FrontIntakeSet Ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
