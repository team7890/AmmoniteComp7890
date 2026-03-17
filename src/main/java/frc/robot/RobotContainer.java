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
import java.util.function.Supplier;
import org.ejml.equation.IntegerSequence.Range;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.PivotIntake;
import frc.robot.commands.ShooterFull;
import frc.robot.commands.TeleopSwerve;
// import frc.robot.generated.TunerConstants_comp;
import frc.robot.subsystems.Hopper.Pivot;
import frc.robot.subsystems.Hopper.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Hopper.Intake;
import frc.robot.commands.LocalSwerve;
import frc.robot.commands.Feed;
// import frc.robot.commands.HubAim;
import frc.robot.commands.ShooterSolo;
import frc.robot.commands.TrenchShotAuto;
// import frc.robot.commands.PhotonDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import java.util.HashMap;


public class RobotContainer {
    /* 
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxSpeed = 1.0 * TunerConstants_beta.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform 
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
    
    */

    CommandXboxController xboxDriver = new CommandXboxController(0);

    private final Supplier<Double> translationAxis = xboxDriver::getLeftY;
    private final Supplier<Double> strafeAxis = xboxDriver::getLeftX;
    private final Supplier<Double> rotationAxis = xboxDriver::getRightX;
    private double MaxSpeed = 1.0 * Constants.Swerve.maxSpeed;


    // Make a swerve subsystem
    private final Swerve s_Swerve = new Swerve();

    // === SUBSYSTEM OBJECTS === \\
    private final Shooter objShooter = new Shooter();
    private final Feeder objFeeder = new Feeder();
    private final Indexer objIndexer = new Indexer();
    private final Intake objIntake = new Intake();
    private final Pivot objPivot = new Pivot();
    // private Photon3 objPhoton3 = new Photon3(drivetrain::addVisionMeasurement);
    private final Field2d field;
  
    public Swerve getSwerve() {
        return s_Swerve;
    }

    // === PathPlanner === \\
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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

        field = new Field2d();
            SmartDashboard.putData("Field", field);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                    s_Swerve,
                    () -> -translationAxis.get(),
                    () -> -strafeAxis.get(),
                    () -> -rotationAxis.get(),
                    () -> false,
                    () -> false
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

    }

    private void configureBindings() {

        // === OFFICIAL CONTROLS === \\

        // == Drive == \\
        //xboxDriver.button(8).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        xboxDriver.button(8).onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //xboxDriver.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.k180deg)));
        xboxDriver.button(7).onTrue(s_Swerve.resetModulesToAbsolute());
        // == Reset Forward Direction

        //xboxDriver.x().whileTrue(drivetrain.applyRequest(() -> brake)); 

        // == Wheels X for no moving

        ///For Testing///
        xboxDriver.rightBumper().whileTrue(
            Commands.runOnce(() -> new LocalSwerve(s_Swerve, getTargetAngle()))  
        );

        // === Shooter == \\
        //xboxDriver.axisGreaterThan(3, 0.25).whileTrue(new ShooterFull(objShooter, MotorSpeeds.dShooterRPM, objFeeder, objIndexer, objIntake, objPivot));
        xboxDriver.leftBumper().whileTrue
            (
                Commands.runOnce(() -> objShooter.runShooter(getShotSpeed()))
            )
            .onFalse
            (
                Commands.runOnce(() -> objShooter.stopShooter())
            );


        // === Intake === \\ 
        xboxDriver.axisGreaterThan(2, 0.25).whileTrue(new RunCommand(
                ()-> objIntake.runIntake(MotorSpeeds.dIntakeSpeed), objIntake));

        xboxDriver.a().toggleOnTrue(new PivotIntake(objPivot, MotorSpeeds.dPivotSpeed));

        xboxDriver.povDown().whileTrue(new RunCommand(
                () -> objIntake.runIntake(-MotorSpeeds.dIntakeSpeed), objIntake));

        xboxDriver.povUp().whileTrue(new RunCommand(
                () -> objFeeder.runFeeder(-MotorSpeeds.dFeederSpeed), objFeeder));
       
       //drivetrain.registerTelemetry(logger::telemeterize);

       // Logger.recordOutput("Swerve/Pose", getPose());
      
    }
 
    public double getTargetAngle() {
        Translation2d target = Constants.Landmarks.hubPosition;
        Pose2d robotPose = Swerve.flipIfRed(s_Swerve.getRobotPose());
        SmartDashboard.putString("angle rpose", robotPose.toString());

        if (robotPose.getX() > Constants.Landmarks.trenchline) {
            if (robotPose.getY() < Constants.Landmarks.yMidline) {
                target = Constants.Landmarks.lowerPassTarget;
            }
            else {
                target = Constants.Landmarks.lowerPassTarget;
            }
        }

        Translation2d toTarget = robotPose.getTranslation().minus(target);
        double targetAngle = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
        SmartDashboard.putNumber("Target Angle", targetAngle);
        return targetAngle;
    }

    public double getDistance(){
        Pose2d robotPose = Swerve.flipIfRed(s_Swerve.getRobotPose());
        SmartDashboard.putString("Robot pose", robotPose.toString());
        Translation2d target = Constants.Landmarks.hubPosition;
        if (robotPose.getX() > Constants.Landmarks.trenchline) {
            if (robotPose.getY() < Constants.Landmarks.yMidline) {
                target = Constants.Landmarks.lowerPassTarget;
            }
            else {
                target = Constants.Landmarks.lowerPassTarget;
            }
        }

        Translation2d toTarget = robotPose.getTranslation().minus(target);
        double targetDistance = Math.abs(Math.hypot(toTarget.getX(), toTarget.getY()));
        SmartDashboard.putNumber("Target distance", targetDistance);
        return targetDistance;
    }

    private double getShotSpeed(){
        return getDistance() * 0.2;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
