// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.AutoCommands;

// import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;

// /** Add your docs here. */
// public class Autos extends SubsystemBase {
//     private final SendableChooser<Command> autoChooser;
//     public static final Autos instance = new Autos();

//     public Autos(){
 
//     autoChooser = AutoBuilder.buildAutoChooser();
//         SmartDashboard.putData("Auto Chooser", autoChooser);
//         //autoChooser.addOption("Chaos with Trench","Chaos with trench");
//         Command Chaos = AutoBuilder.buildAuto("Take 5");

       


//     }
//  Command Lynk = 
//     Commands.sequence("Chaos with trench",
//     commands.deadline("Intake through first pass",
    
//     command.sequence("",
//     AutoBuilder.followPath("Central travel")
//     )));

// Command Chaos = 
//      Commands.sequence(("Sprint"),
    // AutoBuilder.followPath("Central travel"));
// }
