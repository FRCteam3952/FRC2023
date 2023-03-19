package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

import java.util.HashMap;
import java.util.Map;

public class NetworkTablesUtil {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    private static final Map<String, GenericPublisher> publishers = new HashMap<>();
    private static final Map<String, GenericSubscriber> subscribers = new HashMap<>();

    /**
     * Gets the NetworkTables Instance being used by the program
     *
     * @return {@link NetworkTableInstance} used
     */
    public static NetworkTableInstance getNTInstance() {
        return INSTANCE;
    }

    /**
     * Returns the table reference from NetworkTables
     *
     * @param tableName The name of the table
     * @return {@link NetworkTable} corresponding
     */
    public static NetworkTable getTable(String tableName) {
        return INSTANCE.getTable(tableName);
    }

    public static void setLimelightPipeline(int pipeline) {
        NetworkTable table = INSTANCE.getTable("limelight");
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public static int getLimeLightPipeline() {
        NetworkTable table = INSTANCE.getTable("limelight");
        return table.getEntry("getpipe").getNumber(1).intValue();
    }

    public static float getLimeLightErrorX() {
        NetworkTable table = INSTANCE.getTable("limelight");
        if (getLimeLightPipeline() == 1) {
            return table.getEntry("llpython").getNumberArray(new Number[]{0, 0, 0, 0})[1].floatValue() - 160.0f;
        } else {
            return table.getEntry("tx").getNumber(0.0).floatValue() * 5.369f;
        }

    }

    public static float getLimelightTX() {
        return getEntry("limelight", "tx").getNumber(0.0).floatValue();
    }

    public static float getLimeLightErrorY() {
        NetworkTable table = INSTANCE.getTable("limelight");
        if (getLimeLightPipeline() == 1) {
            return table.getEntry("llpython").getNumberArray(new Number[]{0, 0, 0, 0})[2].floatValue() - 120.0f;
        } else {
            return table.getEntry("ty").getNumber(0.0).floatValue() * 5.2516f;
        }
    }

    public static float getLimeLightArea() {
        NetworkTable table = INSTANCE.getTable("limelight");
        return table.getEntry("llpython").getNumberArray(new Number[]{0, 0, 0, 0})[3].floatValue();
    }

    public static float getConeOrientation() {
        NetworkTable table = INSTANCE.getTable("limelight");
        return table.getEntry("llpython").getNumberArray(new Number[]{0, 0, 0, 0})[0].floatValue();
    }

    // Gets key from keyboard
    public static String getKeyString() {
        NetworkTable table = INSTANCE.getTable("robogui");
        return table.getEntry("key_string").getString("default");
    }

    // Gets key from keyboard
    public static int getKeyInteger() {
        NetworkTable table = INSTANCE.getTable("robogui");
        return (int) table.getEntry("key_int").getNumber(0).intValue();
    }

    /**
     * Returns the current robot pose according to AprilTags on Jetson, in meters since that's what they want. The rotation is really the gyro's rotation, since we know that the gyro is accurate.
     *
     * @return A {@link Translation2d} representing the robot's pose ([x, y, radians])
     */
    public static Pose2d getJetsonPoseMeters() {
        NetworkTable table = INSTANCE.getTable("jetson");
        double[] jetsonPoseXYZ = MathUtil.inchesArrayToMetersArray(table.getEntry("pose").getDoubleArray(new double[]{0.0, 0.0, 0.0})); // X, Y, Z
        return new Pose2d(jetsonPoseXYZ[2], jetsonPoseXYZ[0], RobotGyro.getRotation2d());
    }

    public static boolean jetsonHasPose() {
        return getEntry("jetson", "has_pose").getBoolean(false);
    }


    public static boolean getIfOnBlueTeam() {
        NetworkTable table = INSTANCE.getTable("FMSInfo");
        return !table.getEntry("IsRedAlliance").getBoolean(true);
    }

    /**
     * Returns the entry reference from NetworkTables
     *
     * @param tableName Name of the table
     * @param entryName Name of the entry
     * @return {@link NetworkTableEntry} corresponding
     */
    public static NetworkTableEntry getEntry(String tableName, String entryName) {
        return getTable(tableName).getEntry(entryName);
    }

    public static GenericPublisher getPublisher(String tableName, String entryName) {
        String path = "/" + tableName + "/" + entryName;
        var temp = publishers.get(path);
        if (temp != null) {
            return temp;
        }
        var entry = getEntry(tableName, entryName);
        var newPublisher = entry.getTopic().genericPublish(entry.getType().getValueStr(), PubSubOption.keepDuplicates(true));
        publishers.put(path, newPublisher);
        return newPublisher;
    }

    public static GenericSubscriber getSubscriber(String tableName, String entryName) {
        String path = "/" + tableName + "/" + entryName;
        var temp = subscribers.get(path);
        if (temp != null) {
            return temp;
        }
        var entry = getEntry(tableName, entryName);
        var newSubscriber = entry.getTopic().genericSubscribe(entry.getType().getValueStr(), PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
        subscribers.put(path, newSubscriber);
        return newSubscriber;
    }

    public static void getConnections() {
        for (ConnectionInfo connection : INSTANCE.getConnections()) {
            System.out.println("Connection: Using version " + connection.protocol_version + ", ID: " + connection.remote_id + ", IP: " + connection.remote_ip + ", last update: " + connection.last_update);
        }
        if (INSTANCE.getConnections().length == 0) {
            System.out.println("NO CONNECTIONS FOUND");
        }
        System.out.println("END CONNECTIONS LIST \n\n\n\n");
    }

    /**
     * Use in conjunction w/ latencyTest() in network_tables.py to test latency.
     * (Also good example code)
     */
    public static void latencyTesterPeriodicRun() {
        var trajectorySub = getSubscriber("test", "test");
        final var EMPTY = new double[]{};

        TimestampedDoubleArray tsDA = new TimestampedDoubleArray(NetworkTablesJNI.now(), trajectorySub.getLastChange(), trajectorySub.getDoubleArray(EMPTY));
        var timeDiff = (tsDA.timestamp - tsDA.serverTime) / 1000;
        if (timeDiff > 1000) {
            System.out.println(timeDiff - 1000);
        }
    }
}
