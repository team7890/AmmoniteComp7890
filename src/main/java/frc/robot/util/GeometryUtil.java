package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class GeometryUtil {
    public static boolean isNear (Rotation2d expected, Rotation2d acutal, Angle tolerance)  {
        final double expectedRadians = MathUtil.angleModulus(expected.getRadians());
        final double actualRadians = MathUtil.angleModulus(acutal.getRadians());
        return MathUtil.isNear(expectedRadians, actualRadians, 
            tolerance.in(Radians), -Math.PI, Math.PI);
    }
}
