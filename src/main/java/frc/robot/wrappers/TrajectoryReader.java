package frc.robot.wrappers;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericSubscriber;

public class TrajectoryReader {
    public Trajectory currentTrajectory;
    private final GenericSubscriber trajectorySub;
    private static final double[] EMPTY = new double[] {};

    public TrajectoryReader(String trajectoryTable, String trajectoryEntry) {
        this.trajectorySub = NetworkTables.getSubscriber(trajectoryTable, trajectoryEntry);
    }

    public void periodic() {
        var latest = this.trajectorySub.getDoubleArray(EMPTY);
        if(latest.length == 0 || latest.length % 7 != 0) {
            System.out.println("Bad array");
        }

        ArrayList<Trajectory.State> states = new ArrayList<>();

        for(int i = 0; i < latest.length; i++) {
            int shift = i * 7;
            states.add(new Trajectory.State(latest[shift + 0], latest[shift + 1], latest[shift + 2], new Pose2d(latest[shift + 3], latest[shift + 4], new Rotation2d(latest[shift + 5])), latest[shift + 6]));
        }

        this.currentTrajectory = new Trajectory(states);
    }
}