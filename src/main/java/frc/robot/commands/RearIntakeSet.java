// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.IntakeSubsystem;

// public class RearIntakeSet extends InstantCommand {

//     private final IntakeSubsystem intakeSubsystem;
//     private final Boolean on;

//     public RearIntakeSet(IntakeSubsystem intakeSubsystem, Boolean on) {
//         this.on = on;
//         this.intakeSubsystem = intakeSubsystem;
//         addRequirements(intakeSubsystem);
//     }

//     @Override
//     public void initialize() {
//         System.out.println("RearIntakeSet Started!");
//         intakeSubsystem.setRearWheels(on);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         System.out.println("RearIntakeSet Ended!");
//     }


// }
