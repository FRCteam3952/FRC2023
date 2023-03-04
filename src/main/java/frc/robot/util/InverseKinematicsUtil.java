package frc.robot.util;
import frc.robot.Constants.ArmConstants;

/**
 * Inverse Kinematics helper for the arm
 */
public final class InverseKinematicsUtil {
    private static double savedPivot1Angle, savedPivot2Angle, savedTurretAngle;

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
        double adjusted_z = z;

        if (MathUtil.distance(x, 0, z, 0) < ArmConstants.MIN_HOR_DISTANCE){ //Makes sure the arm isn't unrealistically close to the base arm segment
            return getSavedAngles();
        }

        double dist3d = MathUtil.distance(0, adjusted_x, 0, adjusted_y, 0, z); // calc distance in 3d from top pivot point
        if(dist3d > ArmConstants.LIMB1_LENGTH + ArmConstants.LIMB2_LENGTH - ArmConstants.MAX_REACH_REDUCTION) { //If distance reach is impossible then just return saved angles
            return getSavedAngles();
        }

        //inverse kinematics but it looks "simple"
        pivot2Angle = MathUtil.lawOfCosinesForAngle(ArmConstants.LIMB1_LENGTH, ArmConstants.LIMB2_LENGTH, dist3d); // a2 is angle between 1st arm segment to 2nd arm segment
        pivot1Angle = MathUtil.angleBetweenLines(0, -1, 0, adjusted_x, adjusted_y, adjusted_z) - MathUtil.lawOfSinesForAngle(pivot2Angle, dist3d, ArmConstants.LIMB2_LENGTH);   // a1 is angle between verticle to 1st arm segment
       
        //if flipped is true, return angles that are "flipped" 
        if(flipped){
            double angleCalc = Math.toDegrees(Math.atan2(adjusted_x, -y + ArmConstants.ORIGIN_HEIGHT));
            double lineAngle = angleCalc < 0 ? 360 + angleCalc : angleCalc;
            pivot1Angle = lineAngle*2 - pivot1Angle;
            pivot2Angle = 360 - pivot2Angle;
            if (pivot1Angle > 350){
                pivot1Angle = 350;
            }
            if(pivot2Angle > 345){
                pivot2Angle = 345;
            }
        }
        else{
            if (pivot1Angle < 10){
                pivot1Angle = 10;
            }
            if(pivot2Angle < 15){
                pivot2Angle = 15;
            }
        }
        

        //turret angle calculations
        double angleCalc = Math.toDegrees(Math.atan2(z, x));
        turretAngle = angleCalc < 0 ? 360 + angleCalc : angleCalc;

        //updating the saved angles
        setSavedAngles(pivot1Angle, pivot2Angle, turretAngle);

        return new double[] {pivot1Angle, pivot2Angle, turretAngle};
    }

    public static double[] getSavedAngles() {
        return new double[] {savedPivot1Angle, savedPivot2Angle, savedTurretAngle};
    }

    public static void setSavedAngles(double a, double b, double c){
        savedPivot1Angle = a;
        savedPivot2Angle = b;
        savedTurretAngle = c;
    }
}