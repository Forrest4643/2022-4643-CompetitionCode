package frc.robot.commands.RearIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;

public class RearIndexerOn extends CommandBase {

    private final IndexerSubsystem IndexerSubsystem;

    public RearIndexerOn(IndexerSubsystem IndexerSubsystem) {
        this.IndexerSubsystem = IndexerSubsystem;
        addRequirements(IndexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("RearIndexer On!");
    }

    @Override
    public void execute() {
        IndexerSubsystem.setRearIndexer(true);

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RearIndexer Off!");
        IndexerSubsystem.setRearIndexer(false);
    }

}
