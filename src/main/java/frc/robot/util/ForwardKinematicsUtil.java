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
        double x = 0.0, y = 0.0, z = 0.0;
        a1 = Math.toRadians(a1);
        a2 = Math.toRadians(a2);
        turretAngle = Math.toRadians(turretAngle);
        // https://github.com/AymenHakim99/Forward-and-Inverse-Kinematics-for-2-DOF-Robotic-arm COPIED
        x = ArmConstants.LIMB1_LENGTH * Math.cos(a1) + ArmConstants.LIMB2_LENGTH * Math.cos(a1 + a2);
        z = ArmConstants.LIMB1_LENGTH * Math.sin(a1) + ArmConstants.LIMB2_LENGTH * Math.sin(a1 + a2);

        // https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python COPIED
        double x2 = Math.cos(turretAngle) * x - Math.sin(turretAngle) * z;
        double z2 = Math.sin(turretAngle) * x + Math.cos(turretAngle) * z;

        // Not copied:
        // Solve for the diagonal distance from origin -> robot arm's (x, z) using LoC
        // Then, use that distance + the known x distance to solve for the height using Pythag.

        // Solve for the distance from origin -> claw
        y = Math.sqrt(Math.pow(MathUtil.lawOfCosinesForSide(ArmConstants.LIMB1_LENGTH, ArmConstants.LIMB2_LENGTH, a2), 2) - x * x);
        return new double[] {x2, y, z2};
    }
}