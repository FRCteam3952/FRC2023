package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;

public class GenerateTrajectory {
    TrajectoryConfig config;
    Pose2d startPoint;
    Pose2d endPoint;

    public GenerateTrajectory() {
        config = new TrajectoryConfig(0, 0);
        startPoint = new Pose2d();
        endPoint = new Pose2d();
    }

    public Trajectory generate() {
        return TrajectoryGenerator.generateTrajectory(startPoint, null, endPoint, config);
    }
}
