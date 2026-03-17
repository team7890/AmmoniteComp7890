// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import java.util.Optional;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.subsystems.swerve.SwerveModuleConstants;
import frc.lib.util.COTSTalonFXSwerveConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double stickDeadband = 0.12;
  public static final CANBus mechCanBus = new CANBus("MechCan");
  public static final CANBus swerveCanBus = new CANBus("Swerve CANivore");


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }

  public static class MotorIDs {
    public static final int iIntake = 13;
    public static final int iIntakePivot = 14;
    public static final int iIndexerLeader = 15;
    public static final int iIndexerFollower = 21;
    public static final int iFeederLeader = 16;
    public static final int iFeederFollower = 17;
    public static final int iHood = 18;
    public static final int iShooterLeader = 19;
    public static final int iShooterFollower = 20;
    //public static final int iClimbR = ;
    //public static final int iClimbL = ;
    public static final int iEncoderPivotID = 22;

  }

  public static class MotorSpeeds{
    public static final double dShooter3M = 3000;
    public static final double dShooterRPM = 3250;
    public static final double dPassingRPM = 5000;
    public static final double dFeederSpeed = 1.0; 
    public static final double dIndexerSpeed = 1.0;
    public static final double dIntakeSpeed = 0.4;
    public static final double dPivotSpeed = 0.2; //change back to 0.25
    public static final double dPivSlow = 0.15;

  }

  public static class MotorPositions{
    public static final double dPivotMax = 90;
    public static final double dPivotMin = 0;
  }

  public static final class Swerve{
    public static final int pigeonID = 0;
    public static final CANBus CanBus = new CANBus("Drivetrain");
    public static final double maxSpeed = 4.5; // 4.5 FOR COMP
    public static final double maxAngularVelocity = 10.0; //10 FOR COMP


    
    /* Center to Center distance of left and right modules in meters. */
    public static final double trackWidth = Units.inchesToMeters(27); 

    /* Center to Center distance of front and rear module wheels in meters. */
    public static final double wheelBase = Units.inchesToMeters(27); 
    /*
    * Swerve Kinematics
    * No need to ever change this unless you are not doing a traditional
    * rectangular/square 4 module swerve
    */
    /*
     
    // Front Left
    private static final int kFrontLeftDriveMotorId = 2;
    private static final int kFrontLeftSteerMotorId = 1;
    private static final int kFrontLeftEncoderId = 3;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.21142578125);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(13.5);
    private static final Distance kFrontLeftYPos = Inches.of(13.5);

    // Front Right
    private static final int kFrontRightDriveMotorId = 4;
    private static final int kFrontRightSteerMotorId = 5;
    private static final int kFrontRightEncoderId = 6;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.456298828125);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(13.5);
    private static final Distance kFrontRightYPos = Inches.of(-13.5);

    // Back Left
    private static final int kBackLeftDriveMotorId = 8;
    private static final int kBackLeftSteerMotorId = 7;
    private static final int kBackLeftEncoderId = 9;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.322021484375);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-13.5);
    private static final Distance kBackLeftYPos = Inches.of(13.5);

    // Back Right
    private static final int kBackRightDriveMotorId = 10;
    private static final int kBackRightSteerMotorId = 11;
    private static final int kBackRightEncoderId = 12;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.227294921875);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-13.5);
    private static final Distance kBackRightYPos = Inches.of(-13.5);
     
     */
        public static final boolean focEnabled = true; 
        public static final boolean isOnCANivore = true;

        public static final COTSTalonFXSwerveConstants chosenModule = 
                COTSTalonFXSwerveConstants.SDS.MK5n
                        .KrakenX60X44(COTSTalonFXSwerveConstants.SDS.MK5n.driveRatios.L7890);

        /* Drivetrain Constants */
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double wheelRadiusMeters = chosenModule.wheelDiameter / 2.0 ;
        
        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 30;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = false;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = false;

        /*
         * These values are used by the          * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; // 0.12 FOR COMP
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        /*
         * These are theorectial values to start with, tune after
         * Kraken FOC-DIS (L1.0): ft/s = 12.9 | m/s = 3.93192
         * Kraken FOC-ENB (L1.0): ft/s = 12.4 | m/s = 3.77952
         * Kraken FOC-DIS (L1.5): ft/s = 14.2 | m/s = 4.32816
         * Kraken FOC-ENB (L1.5): ft/s = 14.2 | m/s = 4.32816
         * Kraken FOC-DIS (L2.0): ft/s = 15.5 | m/s = 4.7244
         * Kraken FOC-ENB (L2.0): ft/s = 15.0 | m/s = 4.572
         * Kraken FOC-DIS (L2.5): ft/s = 17.7 | m/s = 5.39496
         * Kraken FOC-ENB (L2.5): ft/s = 17.1 | m/s = 5.21208
         * Kraken FOC-DIS (L3.0): ft/s = 17.1 | m/s = 5.21208
         * Kraken FOC-ENB (L3.0): ft/s = 16.5 | m/s = 5.0292
         * Kraken FOC-DIS (L3.5): ft/s = 19.5 | m/s = 5.9436
         * Kraken FOC-ENB (L3.5): ft/s = 18.9 | m/s = 5.76072
         * Kraken FOC-DIS (L4.0): ft/s = 20.4 | m/s = 6.21792
         * Kraken FOC-ENB (L4.0): ft/s = 19.7 | m/s = 6.00456
         */


        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.21142578125); //Rotation2d.fromDegrees(90.0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.456298828125);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.322021484375);//-135
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.227294921875);//180
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }    

}

  public static class Vision {
    public static final String sCameraName = "Dragon";
    public static final Transform3d kRobotToCam = 
        new Transform3d(new Translation3d(0.1397, 0.2921, 0.4953), new Rotation3d(0.0, 0.35, 0.443));
    public static final AprilTagFieldLayout kTagLayout = 
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    public static final Matrix<N3, N1> kSingleTagStdDevs = 
      VecBuilder.fill(4.0, 4.0, 8.0);
    public static final Matrix<N3, N1> kMultiTagStdDevs = 
      VecBuilder.fill(0.5, 0.5, 1.0);
  }
  
  
   public class Landmarks {
        public static final Translation2d hubPosition = new Translation2d(4.625594,4.034536);
        public static final double trenchline = Units.inchesToMeters(181.56);
        public static final double yMidline = Units.inchesToMeters(158.32);
        public static final Translation2d lowerPassTarget = new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(50));
        public static final Translation2d upperPassTarget = new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(316.64 - 50));
   }

   /* 
    public static Translation2d hubPosition() {
      final Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
      }
      return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }
   }
   */

   public static class Driving {
    public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
    public static final AngularVelocity kPIDRotaionDeadband = kMaxRotationalRate.times(0.005);
   }









}