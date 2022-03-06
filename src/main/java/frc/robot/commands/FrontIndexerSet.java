package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;

public class FrontIndexerSet extends InstantCommand {

    private final IndexerSubsystem IndexerSubsystem;
    private final Boolean on;

    public FrontIndexerSet(IndexerSubsystem IndexerSubsystem, Boolean on) {
        this.on = on;
        this.IndexerSubsystem = IndexerSubsystem;
        addRequirements(IndexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("FrontIndexerSet Started!");
        IndexerSubsystem.setFrontIndexer(on);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FrontIndexerSet Ended!");
    }


}
