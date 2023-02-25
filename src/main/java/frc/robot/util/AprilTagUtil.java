package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

public final class AprilTagUtil {
    private AprilTagUtil() {
        throw new UnsupportedOperationException("AprilTagUtil is a utility class and cannot be instantiated!");
    }

    public static Pose3d poseFromTag(double[] rel_cords, int tag_id) {
        /* rel_cords.length() = 3, it's an [x,y,z] coordinate pair
        if the field is rectangular, x is dist of apriltag to the right of center of robot,
        z is up, y is vertically upwards
        */
        // tag_id is apriltag number from 1 to 8
        tag_id--;
        double[] abs_cords = new double[3];
        for (int i = 0; i < 3; i++) {
            abs_cords[i] = Constants.AprilTagConstants.tagInfo[tag_id][i] - rel_cords[i];
        }
        Pose3d pose = new Pose3d(abs_cords[0], abs_cords[1], abs_cords[2], null);
        return pose;
    }
}