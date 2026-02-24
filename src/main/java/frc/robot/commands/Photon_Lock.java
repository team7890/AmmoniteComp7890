// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.Photon;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Photon_Lock extends Command {

  private final CommandSwerveDrivetrain objSwerve;
  private final double dMaxSpeed;
  private final double dMaxAngularRate;
 
  private final DoubleSupplier dsDriverLeftX;
  private final DoubleSupplier dsDriverLeftY;
 
  private double dCmdLeftX, dCmdLeftY;

  double photonAimProportional(){
    double kp = 0.01;
    double dragonRotVel;
    
    if (Photon.PhotonYaw() == 7890.0) {
      dragonRotVel = 0.0;
    }
    else {
      dragonRotVel = Photon.PhotonYaw() * kp;
    }

    dragonRotVel *=dMaxAngularRate;

    dragonRotVel *= -1.0;

    return dragonRotVel;
  }

  private SwerveRequest.FieldCentric drive  = new SwerveRequest.FieldCentric()
  .withDeadband(0.02).withRotationalDeadband(0.02) // Add a 2% deadband 10% is CTRE Value
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;
  
  /** Creates a new Photon_Lock. */
  public Photon_Lock(CommandSwerveDrivetrain objSwerve_in, double dMaxSpeed_in, double dMaxAngularRate_in, DoubleSupplier dsDriverLeftX_in, DoubleSupplier dsDriverLeftY_in) {
    
    objSwerve = objSwerve_in;
    dMaxSpeed = dMaxSpeed_in;
    dMaxAngularRate = dMaxAngularRate_in;
    dsDriverLeftX = dsDriverLeftX_in;
    dsDriverLeftY = dsDriverLeftY_in;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    dCmdLeftX = dsDriverLeftX.getAsDouble();
    dCmdLeftY = dsDriverLeftY.getAsDouble();

    SmartDashboard.putNumber("Photon Aim", photonAimProportional());

    objSwerve.setControl(
      drive.withVelocityX(dCmdLeftY).withVelocityX(dCmdLeftX).withRotationalRate(photonAimProportional())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
