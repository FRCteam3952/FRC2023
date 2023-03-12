package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;
import java.util.ArrayList;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CommandGenerator {

    private static ArrayList<CommandGenerator> instances = new ArrayList<CommandGenerator>();

    private Path path;
    private Trajectory trajectory; 
    private Supplier<Command> commandGenerator;

    public static void initializeAll(DriveTrainSubsystem driveTrain)
    {
        for (CommandGenerator instance : instances) {
            instance.initialize(driveTrain);
        }
    }

    public CommandGenerator(String path) 
    {
        this.path = Filesystem.getDeployDirectory().toPath().resolve(path);
        instances.add(this);
    }

    public void initialize(DriveTrainSubsystem driveTrain)
    {
        try {
            this.trajectory = TrajectoryUtil.fromPathweaverJson(this.path);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        }
        this.commandGenerator = () -> driveTrain.generateRamseteCommand(this.trajectory);
    }

    public Command get()
    {
        return this.commandGenerator.get();
    } 
    
}
