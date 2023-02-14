package frc.robot.commands.ArmCommands;
import frc.robot.Constants.PositionConstants;
import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutomaticObjectPlacementCommand extends CommandBase{
    private final ArmSubsystem arm;
    private final NetworkTableInstance inst;
    private final NetworkTable table;
    private final NetworkTableEntry key;

    public AutomaticObjectPlacementCommand(ArmSubsystem arm, FlightJoystick joystick) {
        this.arm = arm;
        this.inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable("robogui");
        this.key = table.getEntry("key"); // Key pressed on keyboard 
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }
    public void getCoordinatesFromKey(){
        double [] coordinates = new double[3];
        int currKey = (int) key.getInteger(1);
        switch (currKey){
            case 1:
                coordinates = PositionConstants.BOTTOM_LEFT_POS;
                break;
            case 2:
                coordinates = PositionConstants.BOTTOM_MIDDLE_POS;
                break;
            case 3:
                coordinates = PositionConstants.BOTTOM_RIGHT_POS;
                break;
            case 4:
                coordinates =  PositionConstants.CENTER_LEFT_POS;
                break;
            case 5:
                coordinates =  PositionConstants.CENTER_MIDDLE_POS;
                break;
            case 6:
                coordinates =  PositionConstants.CENTER_RIGHT_POS;
                break;
            case 7:
                coordinates = PositionConstants.TOP_LEFT_POS;
                break;
            case 8:
                coordinates = PositionConstants.TOP_CENTER_POS;
                break;
            case 9:
                coordinates =  PositionConstants.TOP_RIGHT_POS;
                break;
            default:
                System.out.println("A key within 1-9 was not pressed");
                break;
        }
        arm.setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2]);
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
        getCoordinatesFromKey();
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
