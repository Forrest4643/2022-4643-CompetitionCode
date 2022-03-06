package frc.robot.commands.FrontIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;

public class FrontIndexerOn extends CommandBase {

    private final IndexerSubsystem IndexerSubsystem;

    public FrontIndexerOn(IndexerSubsystem IndexerSubsystem) {
        this.IndexerSubsystem = IndexerSubsystem;
        addRequirements(IndexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("FrontIndexer On!");
    }

    @Override
    public void execute() {
        IndexerSubsystem.setFrontIndexer(true);

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FrontIndexer Off!");
        IndexerSubsystem.setFrontIndexer(false);
    }

}
