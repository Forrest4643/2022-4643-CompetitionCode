// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.IntakeSubsystem;

// public class RearIntakeSet extends CommandBase {

//     private final IntakeSubsystem intakeSubsystem;
//     private final boolean on;

//     public RearIntakeSet(IntakeSubsystem intakeSubsystem, boolean on) {
//         this.on = on;
//         this.intakeSubsystem = intakeSubsystem;
//         addRequirements(intakeSubsystem);
//     }

//     @Override
//     public void initialize() {
//         System.out.println("RearIntakeSet Started!");
//     }
//     @Override
//     public void execute() {
//         intakeSubsystem.setRearWheels(on);
//     }

//     @Override
//     public void end (boolean interrupted) {
//         System.out.println("RearIntakeSet Ended!");
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

// }
