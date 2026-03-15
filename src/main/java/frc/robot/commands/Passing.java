
package frc.robot.commands;

import frc.robot.Constants.MotorSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Hopper.Indexer;
import frc.robot.subsystems.Hopper.Intake;


///MADE FOR PASSING FUEL///

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Passing extends SequentialCommandGroup {
  /** Creates a new ShooterFull. */
  public Passing (Shooter objShooter, Feeder objFeeder, Indexer objIndexer, Intake objIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> objShooter.runShooterRPM(5000.0), objShooter).withTimeout(0.5),
      new ParallelRaceGroup(
        new RunCommand(() -> objShooter.runShooterRPM(5000.0), objShooter),
        new FireNTheHole(objFeeder, objIndexer, objIntake)
      )
    );
  }
}