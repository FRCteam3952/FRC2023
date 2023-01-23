package frc.robot.util;

import frc.robot.Constants.ArmConstants;

public final class InverseKinematicsUtil {
    private static double x_pos, y_pos, z_pos;

    private InverseKinematicsUtil() {
        throw new UnsupportedOperationException("InverseKinematicsUtil is a utility class and cannot be instantiated");
    }

    public static double distance(double x1, double x2, double y1, double y2) { //2d distance calc
        return Math.sqrt(Math.pow(x1-x2,2) + Math.pow(y1-y2,2));
    }

    public static double distance(double x1, double x2, double y1, double y2, double z1, double z2) { //3d distance calc
        return Math.sqrt(Math.pow(x1-x2,2) + Math.pow(y1-y2,2) + Math.pow(z1-z2,2));
    }

    public static double lawOfCosines(double a, double b, double c) { //law of cosines calc
        return Math.toDegrees(Math.acos((Math.pow(a,2) + Math.pow(b,2) - Math.pow(c,2))/(2*(a*b))));
    }

    public static double lawOfSines(double angle, double a, double b) { //law of sines calc
        return Math.toDegrees(Math.asin((b * Math.sin(Math.toRadians(angle))) / a));
    }

    public static double angleBetweenLines(double x1, double y1, double z1, double x2, double y2, double z2){ //angle between lines
        double dotProduct = x1 * x2 + y1 * y2 + z1 * z2;
        return Math.toDegrees(Math.acos(dotProduct/(Math.abs(distance(0,x1,0,y1,0,z1)*distance(0, x2, 0, y2,0,z2)))));
    }

    /**
     * calculate arm angles relative to limb that it's attached to
     */
    public static double[] getAnglesFromCoordinates(double x, double y, double z) {
        double a1, a2, turretAngle;
        double adjusted_y = y - ArmConstants.ORIGIN_HEIGHT;   // calculate height relative to the origin (at the tip of the non-moving rod which holds the arm)
        double adjusted_x = Math.abs(x);
        double adjusted_z = z;
        double dist3d = distance(0, adjusted_x, 0, adjusted_y, 0, z);     // calc distance in 3d from top pivot point
        double totalLimbLength = ArmConstants.LIMB1_LENGTH + ArmConstants.LIMB2_LENGTH;
        if(dist3d > totalLimbLength) {
            adjusted_x *= (totalLimbLength / dist3d); 
            adjusted_y *= (totalLimbLength / dist3d);
            adjusted_z *= (totalLimbLength / dist3d); 
            x_pos = adjusted_x;
            y_pos = adjusted_y;
            z_pos = adjusted_z;
        }

        if (dist3d == 0) { //zero, zero on coordinate -> prevent divide by 0 exception
            return new double[] {0,0,0};
        }           
        a2 = lawOfCosines(ArmConstants.LIMB1_LENGTH, ArmConstants.LIMB2_LENGTH, dist3d);                                  // a2 is angle between 1st arm segment to 2nd arm segment
        a1 = angleBetweenLines(0, -1, 0, adjusted_x, adjusted_y, adjusted_z) - lawOfSines(a2, dist3d, ArmConstants.LIMB2_LENGTH);   // a1 is angle between verticle to 1st arm segment
       
        //turret angle calculations
        double angleCalc = Math.toDegrees(Math.atan2(z, x));
        turretAngle = angleCalc < 0 ? 360 + angleCalc : angleCalc;
        
        return new double[] {a1, a2, turretAngle};
    }

    public static double[] getCurrentCoordinates() {
        return new double[] {x_pos, y_pos, z_pos};
    }
}