package frc.robot.util;

import frc.robot.Constants.ArmConstants;

public final class InverseKinematicsUtil {
    private static double x_pos, y_pos, z_pos;

    private InverseKinematicsUtil() {
        throw new UnsupportedOperationException("InverseKinematicsUtil is a utility class and cannot be instantiated");
    }

    /**
     * calculate arm angles relative to limb that it's attached to
     */
    public static double[] getAnglesFromCoordinates(double x, double y, double z) {
        double a1, a2, turretAngle;
        double adjusted_y = y - ArmConstants.ORIGIN_HEIGHT;   // calculate height relative to the origin (at the tip of the non-moving rod which holds the arm)
        double adjusted_x = Math.abs(x);
        double adjusted_z = z;
        double dist3d = MathUtil.distance(0, adjusted_x, 0, adjusted_y, 0, z);     // calc distance in 3d from top pivot point
        x_pos = x;
        y_pos = y;
        z_pos = z;
        double totalLimbLength = ArmConstants.LIMB1_LENGTH + ArmConstants.LIMB2_LENGTH;
        if(dist3d >= totalLimbLength - ArmConstants.DISTANCE_DELTA) {
            adjusted_x *= ((totalLimbLength - ArmConstants.DISTANCE_DELTA) / dist3d); 
            adjusted_y *= ((totalLimbLength - ArmConstants.DISTANCE_DELTA) / dist3d);
            adjusted_z *= ((totalLimbLength - ArmConstants.DISTANCE_DELTA) / dist3d); 
            x_pos *= ((totalLimbLength - ArmConstants.DISTANCE_DELTA) / dist3d); 
            y_pos = adjusted_y * ((totalLimbLength - ArmConstants.DISTANCE_DELTA) / dist3d) + ArmConstants.ORIGIN_HEIGHT;
            z_pos *= ((totalLimbLength - ArmConstants.DISTANCE_DELTA) / dist3d);
        }

        if (dist3d == 0) { //zero, zero on coordinate -> prevent divide by 0 exception
            return new double[] {0,0,0};
        }           
        a2 = MathUtil.lawOfCosinesForAngle(ArmConstants.LIMB1_LENGTH, ArmConstants.LIMB2_LENGTH, dist3d);                                  // a2 is angle between 1st arm segment to 2nd arm segment
        a1 = MathUtil.angleBetweenLines(0, -1, 0, adjusted_x, adjusted_y, adjusted_z) - MathUtil.lawOfSinesForAngle(a2, dist3d, ArmConstants.LIMB2_LENGTH);   // a1 is angle between verticle to 1st arm segment
       
        //turret angle calculations
        double angleCalc = Math.toDegrees(Math.atan2(z, x));
        turretAngle = angleCalc < 0 ? 360 + angleCalc : angleCalc;
        
        return new double[] {a1, a2, turretAngle};
    }

    public static double[] getCurrentCoordinates() {
        return new double[] {x_pos, y_pos, z_pos};
    }
}