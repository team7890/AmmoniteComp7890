// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualFire extends SequentialCommandGroup {
  /** Creates a new ManualFire. */
  public ManualFire(Feeder objFeeder, Shooter objShooter, double dShootRPM, double dFeedSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> objShooter.runShooterRPM(dShootRPM), objShooter).withTimeout(0.5),
      new ParallelCommandGroup(
        new RunCommand(() -> objShooter.runShooterRPM(dShootRPM), objShooter),
        new RunCommand(() -> objFeeder.runFeeder(dFeedSpeed), objFeeder)
      )
      //test
    );
  }
}
