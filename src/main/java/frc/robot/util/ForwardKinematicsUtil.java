package frc.robot.util;

import frc.robot.Constants.ArmConstants;

public final class ForwardKinematicsUtil {
    private ForwardKinematicsUtil() {
        throw new UnsupportedOperationException("ForwardKinematicsUtil is a utility class and cannot be instantiated!");
    }

    /**
     * a1, a2, turretAngle in degrees!!!
     */
    public static double[] getCoordinatesFromAngles(double a1, double a2, double turretAngle) {
        double x = 0.0, y = ArmConstants.ORIGIN_HEIGHT, z = 0.0, dist = 0.0;
        double angleSegment2 = Math.toRadians(90 + a1 - a2); 
        double a1Rad = Math.toRadians(a1);
        turretAngle = Math.toRadians(turretAngle);

        //redid forward kinematics to match inverse kinematics
        dist = ArmConstants.LIMB1_LENGTH * Math.sin(a1Rad) + ArmConstants.LIMB2_LENGTH * Math.cos(angleSegment2);
        z = Math.sin(turretAngle) * dist;
        x = Math.cos(turretAngle) * dist;
        y += a1 > 90? ArmConstants.LIMB1_LENGTH * Math.cos(a1Rad) : ArmConstants.LIMB1_LENGTH * Math.cos(a1Rad) * -1;
        y += ArmConstants.LIMB2_LENGTH * Math.sin(angleSegment2);

        // Solve for the distance from origin -> claw
        return new double[] {x, y, z};
    }
}