package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public final class PoseUtil {
    private PoseUtil() {
        throw new UnsupportedOperationException("PoseUtil is a utility class and cannot be instantiated!");
    }
    
    /**
     * Converts a double array to a Translation2d.
     * @param arr A double array in the format [x, y, radians]
     * @return A {@link Translation2d} from the given array
     */
    public static Translation2d doubleArrayToTranslation2d(double[] arr) {
        return new Translation2d(arr[0], arr[1]);
    }
}
