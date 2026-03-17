package frc.robot.subsystems.swerve;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;



public class Swerve extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();
    static final double ODOMETRY_FREQUENCY = 100.0; // Hz

    private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

    public final PIDController xPID, yPID, rPID; 

    public final double positionIZone = 4;
    public final double rotationIZone = 4;
    public final double positionKS = 0.02;
    public final double rotationKS = 0.02;

    public SwerveModule[] mSwerveMods;
    public boolean doRejectUpdate = false;

    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d field;

    private Pose2d simPose = new Pose2d();
    private Pose2d robotPose = new Pose2d();
    private final Pigeon2 gyro;
    private final Vision vision = new Vision();

    public Swerve() {

        double kPx = 0.025;
        double kIx = 0;
        double kDx = 0;
        double kPr = 0.015;
        double kIr = 0;
        double kDr = 0;

        double positionTolerance = 0.75;
        double rotationTolerance = 1.50;
        xPID = new PIDController(kPx, kIx, kDx); 
        yPID = new PIDController(kPx, kIx, kDx);
        rPID = new PIDController(kPr, kIr, kDr); 
        xPID.setTolerance(positionTolerance);
        yPID.setTolerance(positionTolerance);
        rPID.setTolerance(rotationTolerance);
    
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.swerveCanBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        m_poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );    
       
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose,
                    this::setPose,
                    this::getSpeeds,
                    (speeds, feedforwards) -> driveRobotRelativeAuto(speeds),
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    Robot::isRed,
                    this);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }

        SmartDashboard.putData("Gyro Heading", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Gyro");
                builder.addDoubleProperty("Value", () -> getHeading().getDegrees(), null);
            }
        });

        SmartDashboard.putData(
        "Swerve Visualizer",
        builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle", () -> mSwerveMods[0].getState().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond * 20, null);

            builder.addDoubleProperty(
                "Front Right Angle", () -> mSwerveMods[1].getState().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond * 20, null);

            builder.addDoubleProperty(
                "Back Left Angle", () -> mSwerveMods[2].getState().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond * 20, null);

            builder.addDoubleProperty(
                "Back Right Angle", () -> mSwerveMods[3].getState().angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> mSwerveMods[3].getState().speedMetersPerSecond * 20, null);

            builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
        });
    }

    public void drive(double xSpeed, double ySpeed, Translation2d translation, double rotation, boolean fieldRelative, boolean rateLimit, boolean useCoralLimelight) {
        SwerveModuleState[] swerveModuleStates;

        swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public void drive(double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates;

        swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0,
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                0,
                                0,
                                rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        if (Robot.isSimulation()){
            return simPose;
        }
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        simPose = pose;
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeAuto(ChassisSpeeds desirChassisSpeeds) {
        lastCommandedSpeeds = desirChassisSpeeds;
        driveRobotRelative(desirChassisSpeeds, false);
    }

    public void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean isOpenLoop) {
        ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void alignStraight() {
        SwerveModuleState aligned = new SwerveModuleState(0.0, new Rotation2d());

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(aligned, false);
        }
    }

      /**
   * Sets the wheels into an X formation to prevent movement.
   
    public void setX() {
        for (SwerveModule mod : mSwerveMods) {
            mSwerveMods[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        }
    }
    */

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public double getYawRate() {
        return gyro.getAngularVelocityZWorld().getValueAsDouble();
    }

    public void setHeading(Rotation2d heading) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public static Translation2d flipIfRed(Translation2d position) {
        return Robot.isRed() ? FlippingUtil.flipFieldPosition(position) : position;
    }

    public static Pose2d flipIfRed(Pose2d pose) {
        return Robot.isRed() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public static Rotation2d flipIfRed(Rotation2d rotation) {
        return Robot.isRed() ? FlippingUtil.flipFieldRotation(rotation) : rotation;
    }

    public void zeroHeading() {
        if (Robot.isRed()) {
            gyro.setYaw(180);
        }
        gyro.reset();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public Command resetModulesToAbsolute() {
        return Commands.runOnce(
                () -> {
                    for (SwerveModule mod : mSwerveMods) {
                        mod.resetToAbsolute();
                    }
                },
                this);
    }

    public double visionDifference() {
        Pose2d vPose = vision.lastPose();
        if (vPose == null) {
            return Double.POSITIVE_INFINITY;
        }
        double diff = getPose().getTranslation().getDistance(vPose.getTranslation());
        SmartDashboard.putNumber("Vision Difference", diff);
        return diff;
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }
     
    @Override
    public void periodic() {
        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        if (Robot.isSimulation()) {
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(lastCommandedSpeeds, simPose.getRotation());
            simPose = new Pose2d(
                simPose.getX() + fieldSpeeds.vxMetersPerSecond * 0.02,
                simPose.getY() + fieldSpeeds.vyMetersPerSecond * 0.02,
                simPose.getRotation().plus(Rotation2d.fromRadians(fieldSpeeds.omegaRadiansPerSecond * 0.02)));
            SmartDashboard.putString("SimPoseRaw", simPose.toString());

            // Drive sim vision with the simulated pose
            vision.simulationPeriodic(simPose);
        }

        // FIX 3: Call getEstimatedGlobalPose() exactly once per loop and reuse the result.
        var visionEst = vision.getEstimatedGlobalPose();
        SmartDashboard.putString("Vision Est", visionEst.toString());

        // FIX 4: Pass the tuned std devs so the pose estimator weights vision correctly.
        visionEst.ifPresent(est -> {
            var estStdDevs = vision.getEstimationStdDevs();
            m_poseEstimator.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    estStdDevs);
        });

        SmartDashboard.putData("Gyro Data", gyro);
        SmartDashboard.putNumber("Gyro Yaw", getGyroYaw().getDegrees());
        SmartDashboard.putBoolean("is red?", Robot.isRed());
        //Pose2d pose = getPose();
        robotPose = getPose();
        field.setRobotPose(robotPose);   
        SmartDashboard.putString("actual pose", robotPose.toString());
        SmartDashboard.putBoolean("Align/x at set", xPID.atSetpoint());
        SmartDashboard.putBoolean("Align/y at set", yPID.atSetpoint());
        SmartDashboard.putBoolean("Align/r at set", rPID.atSetpoint());
        SmartDashboard.putBoolean("Align/viz at set", visionDifference() < Units.inchesToMeters(1.5));

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Swerve/Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        } 
    }
}