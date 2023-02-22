package frc.robot.util;
// package Constants;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

public class AprilTagUtil {
    public Pose3d pose_from_tag(double [] rel_cords, int tag_id) {
        /* rel_cords.length() = 3, it's an [x,y,z] coordinate pair
        if the field is rectangular, x is dist of apriltag to the right of center of robot,
        z is up, y is vertically upwards
        */
        // tag_id is apriltag number from 1 to 8
        tag_id = tag_id - 1;
        double[] abs_cords = {Constants.AprilTagConstants.tagInfo[tag_id][0] - rel_cords[0],
            Constants.AprilTagConstants.tagInfo[tag_id][1] - rel_cords[1],
            Constants.AprilTagConstants.tagInfo[tag_id][2] - rel_cords[2]

        };
        Pose3d pose = new Pose3d(abs_cords[0], abs_cords[1], abs_cords[2], null);
        return pose;

    }

}