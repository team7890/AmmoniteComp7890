package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public class DriveInputSmoother {
    private static final double kJoysticDeadband = 0.15;
    private static final double kCurveExponent = 1.5;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rotationInput;

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput, DoubleSupplier rotationInput) {
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.rotationInput = rotationInput;
        }

        public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput) {
            this (forwardInput, leftInput, () -> 0);
        }

    public ManualDriveInput getSmoothInput() {
        
        
        
        final Vector<N2> rawTranslationInput = VecBuilder.fill(
            forwardInput.getAsDouble(),leftInput.getAsDouble());
        final Vector<N2> deadbandTranslationInput = 
            MathUtil.applyDeadband(rawTranslationInput, kJoysticDeadband);
        final Vector<N2> curvedTranslationInput =   
            MathUtil.copyDirectionPow(deadbandTranslationInput, kCurveExponent);

        final double rawRotationInput = rotationInput.getAsDouble();
        final double deadbandedRotaionInput = 
            MathUtil.applyDeadband(rawRotationInput, kJoysticDeadband);
        final double curvedRotationInput = 
            MathUtil.copyDirectionPow(deadbandedRotaionInput, kCurveExponent);

        return new ManualDriveInput(
            curvedTranslationInput.get(0),
            curvedTranslationInput.get(1),
            curvedRotationInput
        );
    }
}
