package frc.robot.wrappers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    
    /**
     * Gets the NetworkTables Instance being used by the program
     * @return {@link NetworkTableInstance} used
     */
    public static NetworkTableInstance getInstance() {
        return INSTANCE;
    }

    /**
     * Returns the table reference from NetworkTables
     * @param tableName The name of the table
     * @return {@link NetworkTable} corresponding
     */
    public static NetworkTable getTable(String tableName) {
        return INSTANCE.getTable(tableName);
    }

    /**
     * 
     * @param tableName
     * @param entryName
     * @return
     */
    public static NetworkTableEntry getEntry(String tableName, String entryName) {
        if(tableName == "") {
            return INSTANCE.getEntry(entryName);
        }
        return INSTANCE.getTable(tableName).getEntry(entryName);
    }
}
