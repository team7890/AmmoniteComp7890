package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Landmarks;
import frc.robot.Constants.Driving;

import frc.robot.util.DriveInputSmoother;
import frc.robot.util.GeometryUtil;
import frc.robot.util.ManualDriveInput;

public class HubAim extends Command {

    private static final Angle kAimTolerance = Degrees.of(5);

    private final CommandSwerveDrivetrain objSwerve;
    private final DriveInputSmoother objInputSmoother;

    private DoubleSupplier dsForwardInput, dsLeftInput;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withRotationalDeadband(1.0 / 72.0)
    .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
    .withHeadingPID(5,0,0);

    public HubAim(CommandSwerveDrivetrain objSwerve_in, DoubleSupplier dsForwardInput_in, DoubleSupplier dsLeftInput_in) {
        objSwerve = objSwerve_in;
        dsForwardInput = dsForwardInput_in;
        dsLeftInput = dsLeftInput_in;
        objInputSmoother = new DriveInputSmoother(dsForwardInput, dsLeftInput);
        addRequirements(objSwerve);
    }

    public HubAim(CommandSwerveDrivetrain objSwerve) {
        this(objSwerve, () -> 0, () -> 0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        final ManualDriveInput input = objInputSmoother.getSmoothInput();
        objSwerve.setControl(
            fieldCentricFacingAngleRequest
            .withVelocityX(Driving.kMaxSpeed.times(input.forward))
            .withVelocityY(Driving.kMaxSpeed.times(input.left))
            .withTargetDirection(getDirectionToHub())

        );
        SmartDashboard.putNumber("Dist to hub", getDistanceToHub());
    }

    @Override
    public void end (boolean interrupted)   { }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = 
        objSwerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective
        .rotateBy(objSwerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading,currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = objSwerve.getState()
            .Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = 
            hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = 
            hubDirectionInBlueAlliancePerspective
            .rotateBy(objSwerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

    private double getDistanceToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = objSwerve.getState().Pose.getTranslation();
        final double dDistance = hubPosition.minus(robotPosition).getDistance(robotPosition);
        return dDistance;
    }
}
