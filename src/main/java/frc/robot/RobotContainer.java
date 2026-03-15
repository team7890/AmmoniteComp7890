// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.ejml.equation.IntegerSequence.Range;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.commands.Photon_Lock;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.ShooterFull;
import frc.robot.generated.TunerConstants;
// import frc.robot.generated.TunerConstants_comp;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper.Pivot;
import frc.robot.subsystems.Hopper.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Hopper.Intake;

import frc.robot.commands.Feed;
import frc.robot.commands.HubAim;
import frc.robot.commands.ShooterSolo;
import frc.robot.commands.TrenchShotAuto;
// import frc.robot.commands.PhotonDrive;
import frc.robot.commands.PhoNewRot;
// import frc.robot.commands.PhotonShooter;
import frc.robot.Photon;
import frc.robot.Photon2;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import java.util.HashMap;

// import frc.robot.subsystems.Photon3;



public class RobotContainer {
     private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxSpeed = 1.0 * TunerConstants_beta.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // === CONTROLLERS === \\
    private final CommandXboxController xboxDriver = new CommandXboxController(0);

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants_beta.createDrivetrain(); 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // comment out the other one for the comp chassi

    // === SUBSYSTEM OBJECTS === \\
    private final Shooter objShooter = new Shooter();
    private final Feeder objFeeder = new Feeder();
    private final Indexer objIndexer = new Indexer();
    private final Intake objIntake = new Intake();
    private final Pivot objPivot = new Pivot();
    private final Photon2 objPhoton = new Photon2();
    // private Photon3 objPhoton3 = new Photon3(drivetrain::addVisionMeasurement);
    private final Field2d field;
  

    // === PathPlanner === \\
    private final SendableChooser<Command> autoChooser;
 

    
    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        //autoChooser.addOption("Chaos with Trench","Chaos with trench");
        // Command Chaos = AutoBuilder.buildAuto("Take 5");

        // autoChooser.addOption("Take 5", Chaos);

        NamedCommands.registerCommand("stop Shooter", getAutonomousCommand());
        NamedCommands.registerCommand("Intake Pivot", getAutonomousCommand());
        NamedCommands.registerCommand("Print Message", getAutonomousCommand());
        NamedCommands.registerCommand("Run Intake", getAutonomousCommand());
        NamedCommands.registerCommand("stop Shooter", new ShooterFull(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));
        NamedCommands.registerCommand("Run Intake", new RunCommand(()-> objIntake.runIntake(MaxSpeed)).withTimeout(5.0));
        NamedCommands.registerCommand("Auto shots", new TrenchShotAuto(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));
        NamedCommands.registerCommand("Auto Shoot", new TrenchShotAuto(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));
        
        new EventTrigger("Run Intake").whileTrue(new RunCommand(()-> objIntake.runIntake(MotorSpeeds.dIntakeSpeed), objIntake));
        //new EventTrigger("Auto Shoot").whileTrue(new RunCommand(()-> objShooter.runShooterRPM(MotorSpeeds.dShooter3M), objShooter));
        new EventTrigger("Intake Pivot").whileTrue(new PivotIntake(objPivot, MotorSpeeds.dPivotSpeed));
        new EventTrigger("Shooter Run").whileTrue(new ShooterFull(objShooter, 2500, objFeeder, objIndexer, objIntake, objPivot));
        new EventTrigger("Shoot Slower").whileTrue(new ShooterFull(objShooter, 3300, objFeeder, objIndexer, objIntake, objPivot));

        configureBindings();

      

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        // SmartDashboard.putNumber("null", DriverStation.getAlliance());

  
   

        field = new Field2d();
            SmartDashboard.putData("Field", field);

        
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

                                             ///SET DEFAULTCOMANDS ///
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-xboxDriver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xboxDriver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-xboxDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );





        objFeeder.setDefaultCommand(
            new RunCommand(()-> objFeeder.stopFeeder(), objFeeder)
        );

        objIndexer.setDefaultCommand(
            new RunCommand(()->objIndexer.stopIndexer(), objIndexer)
        );

        objShooter.setDefaultCommand(
            new RunCommand(()->objShooter.runShooter(0.0), objShooter) //Original 0.15
        );

        objPivot.setDefaultCommand(
            new RunCommand(() -> objPivot.stopPivot(), objPivot)
        );

        objIntake.setDefaultCommand(
            new RunCommand(() -> objIntake.stopIntake(), objIntake)
        );
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // xboxDriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // xboxDriver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-xboxDriver.getLeftY(), -xboxDriver.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // xboxDriver.back().and(xboxDriver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // xboxDriver.back().and(xboxDriver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // xboxDriver.start().and(xboxDriver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // xboxDriver.start().and(xboxDriver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on 3-line button press.
        

        // === OFFICIAL CONTROLS === \\

        // == Drive == \\
        xboxDriver.button(8).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        xboxDriver.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.k180deg)));
        // == Reset Forward Direction

        xboxDriver.x().whileTrue(drivetrain.applyRequest(() -> brake)); 
        // == Wheels X for no moving

        ///For Testing///
    
        xboxDriver.rightBumper().whileTrue(new Photon_Lock(drivetrain, objPhoton, MaxSpeed, MaxAngularRate, 
                () -> xboxDriver.getLeftY(), 
                () -> xboxDriver.getLeftX()));

        // xboxDriver.rightBumper().whileTrue(new HubAim(drivetrain));

        // == Lock onto AprilTags


        // === Shooter == \\
        xboxDriver.axisGreaterThan(3, 0.25).whileTrue(new ShooterFull(objShooter, MotorSpeeds.dShooterRPM, objFeeder, objIndexer, objIntake, objPivot));

        

          ///Shoot from 3m away///
        xboxDriver.y().whileTrue(new ShooterFull(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));

          ///Passing shooter///
        xboxDriver. b().whileTrue(new ShooterFull(objShooter, MotorSpeeds.dPassingRPM, objFeeder, objIndexer, objIntake, objPivot));

        // xboxDriver.leftBumper().whileTrue(new PhotonShooter(objPhoton, objShooter, drivetrain,
        //     () -> xboxDriver.getLeftX(),
        //     () -> xboxDriver.getLeftY()));
        // == Shoot via distance from april tag

        // === Intake === \\ 
        xboxDriver.axisGreaterThan(2, 0.25).whileTrue(new RunCommand(
                ()-> objIntake.runIntake(MotorSpeeds.dIntakeSpeed), objIntake));

        xboxDriver.a().toggleOnTrue(new PivotIntake(objPivot, MotorSpeeds.dPivotSpeed));

        xboxDriver.povDown().whileTrue(new RunCommand(
                () -> objIntake.runIntake(-MotorSpeeds.dIntakeSpeed), objIntake));

        xboxDriver.povUp().whileTrue(new RunCommand(
                () -> objFeeder.runFeeder(-MotorSpeeds.dFeederSpeed), objFeeder));



       

    //    objPhoton3.periodic();
       
       drivetrain.registerTelemetry(logger::telemeterize);

       // Logger.recordOutput("Swerve/Pose", getPose());
      
    }

// Pose2d poseA = new Pose2d();
// Pose2d poseB = new Pose2d();

// StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
//   .getStructTopic("MyPose", Pose2d.struct).publish();
// StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
//   .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

// private void periodic() {
//   publisher.set(poseA);
//   arrayPublisher.set(new Pose2d[] {poseA, poseB});
// }
 
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
