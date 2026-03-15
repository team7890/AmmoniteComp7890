// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import frc.robot.subsystems.Photon3;
// import frc.robot.subsystems.Shooter.Shooter;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.commands.Photon_Lock;
// import frc.robot.commands.HubAim;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class PhotonShooter extends Command {
  
//   private Photon3 objPhoton3;
//   private Shooter objShooter;
//   private CommandSwerveDrivetrain objDrive;
//   private HubAim objHubAim;

//   double dMaxSpeed;

//   private final DoubleSupplier dsDriverLeftX;
//   private final DoubleSupplier dsDriverLeftY;

//   /** Creates a new PhotonShooterTestV1. */
//   public PhotonShooter(HubAim objHubAim, Shooter objShooter_in, CommandSwerveDrivetrain objDrive_in, DoubleSupplier dsDriverLeftX_in, DoubleSupplier dsDriverLeftY_in) {
//     objPhoton3 = objPhoton_in;
//     objHubAim = objHubAim_in;
//     objShooter = objShooter_in;
//     objDrive = objDrive_in;

//     dsDriverLeftX = dsDriverLeftX_in;
//     dsDriverLeftY = dsDriverLeftY_in;
    
//     addRequirements(objPhoton3, objShooter, objDrive);
//     // Use addRequirements() here to declare subsystem dependencies.
    
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     //new Photon_Lock(objDrive, 0, 0, , );
//     objShooter.runShooterRPM(
//       objShooter.getDistance2RPM(
//         // objPhoton.PhotonDist()
//         objHubAim.getDistanceToHub()
//       ));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
