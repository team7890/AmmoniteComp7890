// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotIntake extends Command {
  /** Creates a new PivotIntake. */

  private final Pivot objPivot;
  private double dPosition;
  private boolean bMoveUp;
  private boolean bDone;
  private double dSpeed;

  public PivotIntake(Pivot objPivot_in, double dSpeed_in) {
    // Use addRequirements() here to declare subsystem dependencies.
    objPivot = objPivot_in;
    dSpeed = dSpeed_in;
    addRequirements(objPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dPosition = objPivot.getIntakePosition();
    if (dPosition < 0.5) {
      bMoveUp = false;
    }
    else {
      bMoveUp = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  ///Increased speed///

  public void execute() {
    if (bMoveUp) {
      objPivot.runPivot(-dSpeed);
    }
    else {
      objPivot.runPivot(dSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objPivot.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    bDone = false;
    if (bMoveUp && objPivot.getIntakePosition() < 0.35) {
        bDone = true;
    }
    if (!bMoveUp && objPivot.getIntakePosition() > 0.60) {
        bDone = true;
    }
    return bDone;
  }
}
