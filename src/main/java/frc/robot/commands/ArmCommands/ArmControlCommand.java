package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants.ArmInverseKinematicsConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.ArmSubsystem;

import java.util.Math;

/**
 * Moves arm on the turret
 */
public class ArmControlCommand extends CommandBase{
    private final ArmSubsystem arm;
    private final FlightJoystick joystick;
    private final NetworkTableInstance inst;
    private final NetworkTable table;
    private final NetworkTableEntry xError, yError;
    private final double xConst, yConst;

    public ArmControlCommand(ArmSubsystem arm, FlightJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
        this.inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable("vision");
        this.xError = table.getEntry("xError");
        this.yError = table.getEntry("yError");
        this.xConst = 360; //can tune later
        this.yConst = 240; //can tune later

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    private double[] getAdjustmentFromCoords() {
        double[] adjustments = new double[3];
        adjustments[0] = xError; //x
        adjustments[1] = yError; //y
        adjustments[2] = xError; //z
        return adjustments;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (joystick.getRawButtonWrapper(ControllerConstants.AIM_ASSIST_BUTTON_NUMBER)) {
            getAdjustmentFromCoords[0], getAdjustmentFromCoords[1], getAdjustmentFromCoords[2]);
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
