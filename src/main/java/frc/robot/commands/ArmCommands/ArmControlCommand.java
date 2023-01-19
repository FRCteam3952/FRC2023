package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants.ArmInverseKinematicsConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Moves arm on the turret
 */
public class ArmControlCommand extends CommandBase{
    private final ArmSubsystem arm;
    private final FlightJoystick joystick;
    private final NetworkTableInstance inst;
    private final NetworkTable table;
    private final NetworkTableEntry xCoord, yCoord;
    private final double xConst, yConst;

    public ArmControlCommand(ArmSubsystem arm, FlightJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
        this.inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable("vision");
        this.xCoord = table.getEntry("x");
        this.yCoord = table.getEntry("y");
        this.xConst = 1; //can tune later
        this.yConst = 1; //can tune later

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (joystick.getRawButtonWrapper(ControllerConstants.AIM_ASSIST_BUTTON_NUMBER)) {
            arm.setIntendedCoordinates(joystick.getHorizontalMovement() + xCoord.getDouble(0)/xConst, 
                ArmInverseKinematicsConstants.ORIGIN_HEIGHT + yCoord.getDouble(0)/yConst, joystick.getLateralMovement());
        } else {
            arm.setIntendedCoordinates(joystick.getHorizontalMovement(), ArmInverseKinematicsConstants.ORIGIN_HEIGHT, joystick.getLateralMovement());
        }
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
