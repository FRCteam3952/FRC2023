package frc.robot.commands.ArmCommands;
import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutomaticObjectPlacementCommand extends CommandBase{
    private final ArmSubsystem arm;
    private final FlightJoystick joystick;
    private final NetworkTableInstance inst;
    private final NetworkTable table;
    private final NetworkTableEntry key;

    public AutomaticObjectPlacementCommand(ArmSubsystem arm, FlightJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
        this.inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable("robogui");
        this.key = table.getEntry("key"); // Key pressed on keyboard 
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.

    // When 1 pressed -> bottom left
    // When 2 pressed -> bottom middle
    // ...
    @Override
    public void execute() {

    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
