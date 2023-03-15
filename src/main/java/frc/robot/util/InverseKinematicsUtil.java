package frc.robot.util;
import frc.robot.Constants.ArmConstants;

/**
 * Inverse Kinematics helper for the arm
 */
public final class InverseKinematicsUtil {
    private InverseKinematicsUtil() {
        throw new UnsupportedOperationException("InverseKinematicsUtil is a utility class and cannot be instantiated");
    }

    /**
     * calculate arm angles relative to limb that it's attached to
     *
     * @param x       X coordinate
     * @param y       Y coordinate
     * @param z       Z coordinate
     * @param flipped Whether the arm should attempt to approach from above rather than from the side (true for above, false for side)
     * @return The angles: [angle_limb_1, angle_limb_2, turret_angle]
     */
    public static double[] getAnglesFromCoordinates(double x, double y, double z, boolean flipped) {
        
        double pivot1Angle, pivot2Angle, turretAngle;
        double adjusted_y = y - ArmConstants.ORIGIN_HEIGHT;
        double adjusted_x = x;

        if(x < ArmConstants.MIN_HOR_DISTANCE){
            x = ArmConstants.MIN_HOR_DISTANCE;
        }

        // Turret angle calculations
        double angleCalc = Math.toDegrees(Math.atan2(z, x));
        turretAngle = angleCalc < 0 ? 360 + angleCalc : angleCalc;

        // distance reach boundar
        double dist3d = MathUtil.distance(0, adjusted_x, 0, adjusted_y, 0, z); // calc distance in 3d from top pivot point
        if(dist3d > ArmConstants.LIMB1_LENGTH + ArmConstants.LIMB2_LENGTH) { // If distance reach is impossible then just return saved angles
            return new double[]{Double.NaN, Double.NaN, Double.NaN};
        }

        //inverse kinematics but it looks "simple"
        pivot2Angle = MathUtil.lawOfCosinesForAngle(ArmConstants.LIMB1_LENGTH, ArmConstants.LIMB2_LENGTH, dist3d); // pivot2Angle is angle between 1st arm segment to 2nd arm segment
        pivot1Angle = (90 + Math.toDegrees(Math.atan(adjusted_y/MathUtil.distance(adjusted_x, 0, z, 0)))) - MathUtil.lawOfCosinesForAngle(dist3d, ArmConstants.LIMB1_LENGTH, ArmConstants.LIMB2_LENGTH);   // pivot1Angle is angle between verticle to 1st arm segment
       
        // If flipped is true, return angles that are "flipped" 
        if(flipped){
            angleCalc = Math.toDegrees(Math.atan2(adjusted_x, -y + ArmConstants.ORIGIN_HEIGHT));
            double lineAngle = angleCalc < 0 ? 360 + angleCalc : angleCalc;
            pivot1Angle = lineAngle * 2 - pivot1Angle;
            pivot2Angle = 360 - pivot2Angle;
            if (pivot1Angle > 350) {
                pivot1Angle = 350;
            }
            if(pivot2Angle > 340) {
                pivot2Angle = 340;
            }
        }
        else{
            if (pivot1Angle < ArmConstants.ARM_1_INITIAL_ANGLE) {
                pivot1Angle = ArmConstants.ARM_1_INITIAL_ANGLE;
            }
            if(pivot2Angle < ArmConstants.ARM_2_INITIAL_ANGLE) {
                pivot2Angle = ArmConstants.ARM_2_INITIAL_ANGLE;
            }
        }

        if(turretAngle > 180) {
            turretAngle -= 360;
        } else if(turretAngle < -180) {
            turretAngle += 360;
        }

        return new double[] {pivot1Angle, pivot2Angle, turretAngle};
    }
}