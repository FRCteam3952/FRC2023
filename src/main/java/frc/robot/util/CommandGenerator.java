package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CommandGenerator {

    private static String trajectoryFolder = "paths/";
    private static String trajectoryExtension = ".wpilib.json";

    private static ArrayList<CommandGenerator> allCommandGenerators = new ArrayList<CommandGenerator>();

    private Path path;
    private Trajectory trajectory; 
    private Supplier<Command> commandGenerator;

    public static void initializeAll(DriveTrainSubsystem driveTrain)
    {
        for (CommandGenerator commandGenerator : allCommandGenerators) {
            commandGenerator.initialize(driveTrain);
        }
    }

    public CommandGenerator(String name) 
    {
        String path = trajectoryFolder + name + trajectoryExtension; // e.g. "paths/" + "driveForwardOverChargeStationBlue" + .wpilib.json"
        this.path = Filesystem.getDeployDirectory().toPath().resolve(path);

        try {
            this.trajectory = TrajectoryUtil.fromPathweaverJson(this.path);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        }

        allCommandGenerators.add(this);
    }

    public void initialize(DriveTrainSubsystem driveTrain)
    {
        this.commandGenerator = () -> driveTrain.generateRamseteCommand(this.trajectory);
    }

    public Command get()
    {
        return this.commandGenerator.get();
    }
}
