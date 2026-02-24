// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.commands.Photon_Lock;
import frc.robot.commands.ShooterFull;
import frc.robot.generated.TunerConstants;
// import frc.robot.generated.TunerConstants_comp;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper.Extender;
import frc.robot.subsystems.Hopper.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Hopper.Intake;

import frc.robot.commands.FireNTheHole;
import frc.robot.commands.ManualFire;
// import frc.robot.commands.PhotonDrive;

public class RobotContainer {
     private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxSpeed = 1.0 * TunerConstants_beta.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // === CONTROLLERS === \\
    private final CommandXboxController xboxDriver = new CommandXboxController(0);
    private final CommandXboxController xboxOperator = new CommandXboxController(1);

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants_beta.createDrivetrain(); 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // comment out the other one for the comp chassi

    // === SUBSYSTEM OBJECTS === \\
    private final Shooter objShooter = new Shooter();
    private final Feeder objFeeder = new Feeder();
    private final Indexer objIndexer = new Indexer();
    private final Intake objIntake = new Intake();
    private final Extender objExtender = new Extender();

    // === PathPlanner === \\
    private final SendableChooser<Command> autoChooser;

    
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();

        NamedCommands.registerCommand("run Shooter", new RunCommand(() -> objShooter.runShooterRPM(MotorSpeeds.dShooterRPM)));
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
            new RunCommand(()->objShooter.runShooter(0.15), objShooter)
        );

        objExtender.setDefaultCommand(
            new RunCommand(() -> objExtender.stopExtender(), objExtender)
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

        // Reset the field-centric heading on leftbumper press.
        xboxDriver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        xboxDriver.rightBumper().whileTrue(new Photon_Lock(drivetrain, MaxSpeed, MaxAngularRate, 
                () -> xboxOperator.getLeftX(), 
                () -> xboxOperator.getLeftY()));
        


    //    xboxOperator.a().whileTrue(new RunCommand(() -> objShooter.runShooter(MotorSpeeds.dShooterSpeed), objShooter))
    //                    .whileFalse(new RunCommand(() -> objShooter.stopShooter(), objShooter));
        
    //     xboxOperator.b().whileTrue(new RunCommand(() -> objFeeder.runFeeder(MotorSpeeds.dFeederSpeed), objFeeder))
    //                    .whileFalse(new RunCommand(() -> objFeeder.stopFeeder(), objFeeder));

    //    xboxOperator.y().whileTrue(new RunCommand(()-> objIndexer. runIndexer(MotorSpeeds.dIndexerSpeed), objIndexer))
    //                     .whileFalse(new RunCommand(()-> objIndexer.stopIndexer(), objIndexer));



        // xboxDriver.a ().whileTrue(new RunCommand(() -> objShooter.runShooter(MotorSpeeds.dShooterSpeed), objShooter));
                    //    .whileFalse(new RunCommand(() -> objShooter.setDefaultCommand(0.15), objShooter) );
        
        xboxDriver.axisGreaterThan(2, 0.5).whileTrue(new ShooterFull(objShooter, objFeeder, objIndexer,objIntake));

        xboxDriver.rightBumper().whileTrue(new Photon_Lock(drivetrain, MaxSpeed, MaxAngularRate,
            () -> xboxDriver.getLeftX(), 
            () -> xboxDriver.getLeftY()));

        // xboxDriver.rightTrigger().whileTrue(new FireNTheHole(objFeeder, objIndexer, objIntake));

        // xboxDriver.x () .whileTrue(new RunCommand(()-> objIndexer.runIndexer(MotorSpeeds.dIndexerSpeed), objIndexer));
                             
        xboxDriver.leftBumper() . whileTrue(new RunCommand(()-> objIntake.runIntake(MotorSpeeds.dIntakeSpeed),objIntake));
                                // .whileFalse(new RunCommand(()-> objIntake. stopIntake(), objIntake));
            
            // === SHOOTER TESTING === \\
        // xboxDriver.a().whileTrue(new ManualFire(objFeeder, objShooter, MotorSpeeds.dShooterRPM, MotorSpeeds.dFeederSpeed));
        // xboxDriver.x().whileTrue(new ManualFire(objFeeder, objShooter, 2000, 0.5));
        // xboxDriver.y().whileTrue(new ManualFire(objFeeder, objShooter, 3000, MotorSpeeds.dFeederSpeed));
        xboxDriver.a().whileTrue(new RunCommand(() -> objIndexer.runIndexer(0.5), objIndexer));

        xboxDriver.b().whileTrue(new RunCommand(() -> objExtender.runExtender(0.1), objExtender));

        xboxOperator.axisGreaterThan(2, 0.5).whileTrue(new ShooterFull(objShooter, objFeeder, objIndexer, objIntake));

        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );

        return autoChooser.getSelected();
    }
}
