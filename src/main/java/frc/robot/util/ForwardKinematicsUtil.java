package frc.robot.util;

import frc.robot.Constants.ArmConstants;

/**
 * Forward Kinematics helper for the arm
 */
public final class ForwardKinematicsUtil {
    private ForwardKinematicsUtil() {
        throw new UnsupportedOperationException("ForwardKinematicsUtil is a utility class and cannot be instantiated!");
    }

    /**
     * Given the angles of the limbs and the turret, find the coordinates
     *
     * @param a1          First limb's angle in degrees
     * @param a2          Second limb's angle in degrees
     * @param turretAngle Turret's angle in degrees
     * @return the coordinates x, y, z
     */
    public static double[] getCoordinatesFromAngles(double a1, double a2, double turretAngle) {
        double z = 0;
        double y = -ArmConstants.LIMB1_LENGTH * Math.cos(Math.toRadians(a1)) + ArmConstants.LIMB2_LENGTH * Math.cos(Math.toRadians(a2 - a1));
        double x = ArmConstants.LIMB1_LENGTH * Math.sin(Math.toRadians(a1)) + ArmConstants.LIMB2_LENGTH * Math.sin(Math.toRadians(a2 - a1));

        // Solve for the distance from origin -> claw
        return new double[]{x, y, z};
    }
}