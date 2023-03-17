package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;

/**
 * Does Math stuff for us
 */
public final class MathUtil {
    private MathUtil() {
        throw new UnsupportedOperationException("MathUtil is a utility class and should not be instantiated!");
    }

    /**
     * Calculates the distance between two points on a 2D plane.
     *
     * @param x1 First point's x coordinate
     * @param x2 Second point's x coordinate
     * @param y1 First point's y coordinate
     * @param y2 Second point's y coordinate
     * @return The distance
     */
    public static double distance(double x1, double x2, double y1, double y2) { //2d distance calc
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    /**
     * Returns the distance between two points on a 3D plane
     *
     * @param x1 First point's x coordinate
     * @param x2 Second point's x coordinate
     * @param y1 First point's y coordinate
     * @param y2 Second point's y coordinate
     * @param z1 First point's z coordinate
     * @param z2 Second point's z coordinate
     * @return The distance
     */
    public static double distance(double x1, double x2, double y1, double y2, double z1, double z2) { //3d distance calc
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2) + Math.pow(z1 - z2, 2));
    }

    /**
     * Solves Law of Cosines for the angle
     *
     * @param a Side a
     * @param b Side b
     * @param c Side c
     * @return Angle opposite c
     */
    public static double lawOfCosinesForAngle(double a, double b, double c) { //law of cosines calc
        return Math.toDegrees(Math.acos((Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * (a * b))));
    }

    /**
     * Solves Law of Cosines for a side
     *
     * @param a                Side a
     * @param b                Side b
     * @param includedAngleDeg Angle opposite the unknown side
     * @return The side you're looking for
     */
    public static double lawOfCosinesForSide(double a, double b, double includedAngleDeg) {
        return Math.sqrt(a * a + b * b - 2 * a * b * Math.cos(Math.toRadians(includedAngleDeg)));
    }

    /**
     * Solves Law of Sines for a side
     *
     * @param angle The angle opposite side A
     * @param a     Side opposite angle
     * @param b     Side opposite unknown angle
     * @return Unknown angle opposite b
     */
    public static double lawOfSinesForAngle(double angle, double a, double b) { //law of sines calc
        return Math.toDegrees(Math.asin((b * Math.sin(Math.toRadians(angle))) / a));
    }

    /**
     * Returns the angle between two points on a 3D plane
     *
     * @param x1 First point's x coordinate
     * @param x2 Second point's x coordinate
     * @param y1 First point's y coordinate
     * @param y2 Second point's y coordinate
     * @param z1 First point's z coordinate
     * @param z2 Second point's z coordinate
     * @return The angle between the points
     */
    public static double angleBetweenLines(double x1, double y1, double z1, double x2, double y2, double z2) { //angle between lines
        double dotProduct = x1 * x2 + y1 * y2 + z1 * z2;
        return Math.toDegrees(Math.acos(dotProduct / (Math.abs(distance(0, x1, 0, y1, 0, z1) * distance(0, x2, 0, y2, 0, z2)))));
    }

    /**
     * Converts inches to meters
     *
     * @param inches inches
     * @return meters
     */
    public static double inchesToMeters(double inches) {
        return inches / 39.37;
    }

    /**
     * Converts an array of inch values to an array of meters values. Returns a new array.
     *
     * @param inchesArray The array of inches
     * @return The array of meters
     */
    public static double[] inchesArrayToMetersArray(double[] inchesArray) {
        double[] metersArray = new double[inchesArray.length];
        for (int i = 0; i < inchesArray.length; i++) {
            metersArray[i] = inchesToMeters(inchesArray[i]);
        }
        return metersArray;
    }

    /**
     * Mirrors a coordinate across the middle of the field's X-axis (for when we are on red alliance).
     *
     * @param value The value to mirror (likely a blue alliance coordinate, but can be either alliance)
     * @return The mirrored value
     * @see {@link #mirrorPoseOnFieldForOppositeSide(Pose3d)}
     */
    public static double mirrorValueOnFieldForOppositeSide(double value) {
        return FieldConstants.FIELD_X_LENGTH - value;
    }

    /**
     * Mirrors the pose across the middle of the field's X-axis (for when we are on red alliance).
     *
     * @param pose
     * @return A new Pose3d with the mirrored value(s).
     * @see {@link #mirrorValueOnFieldForOppositeSide(double)}
     */
    public static Pose3d mirrorPoseOnFieldForOppositeSide(Pose3d pose) {
        return new Pose3d(mirrorValueOnFieldForOppositeSide(pose.getX()), pose.getY(), pose.getZ(), pose.getRotation());
    }

    /**
     * Given the robot's field relative position, and the claw's robot relative position, find the claw's field relative position.
     *
     * @param robotPoseFieldRelative The robot's field relative position.
     * @param clawPoseRobotRelative  The claw's robot relative position (rotation not necessary). This could be anything, but we only use it for the claw.
     * @return The claw's field relative position
     */
    public static Pose2d findFieldRelativePose(Pose2d robotPoseFieldRelative, Pose2d clawPoseRobotRelative) {
        double robotX = robotPoseFieldRelative.getX(), robotY = robotPoseFieldRelative.getY();
        double clawX = clawPoseRobotRelative.getX(), clawY = clawPoseRobotRelative.getY();

        Rotation2d robotAngle = robotPoseFieldRelative.getRotation();
        double newClawX = clawX * robotAngle.getCos() - clawY * robotAngle.getSin();
        double newClawY = clawX * robotAngle.getSin() + clawY * robotAngle.getCos();

        double fieldClawX = newClawX + robotX;
        double fieldClawY = newClawY + robotY;
        return new Pose2d(fieldClawX, fieldClawY, new Rotation2d());
    }
}