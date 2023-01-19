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
    private final NetworkTableEntry xError, yError, area;
    private final double xConst, yConst;
    private final double xSpeed, ySpeed, zSpeed;
    private final double DESIRED_AREA = 50; // can tune later

    public ArmControlCommand(ArmSubsystem arm, FlightJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
        this.inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable("vision");
        this.xError = table.getEntry("xError"); // distance along the x-axis between center of camera vision and center of detected gamepiece
        this.yError = table.getEntry("yError"); // distance along the y-axis between center of camera vision and center of detected gamepiece
        this.area = table.getEntry("area"); // area of gamepiece from perspective of camera vision
        this.xConst = 360; // can tune later
        this.yConst = 240; // can tune later
        this.zConst = 60; //can tune later
        this.xSpeed = 0.1; // inches per 20ms
        this.ySpeed = 0.1; // inches per 20ms
        this.zSpeed = 0.1; // inches per 20ms

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // figure this out later
    private double[] getAdjustmentFromError() {
        double[] adjustments = new double[3];
        adjustments[0] = xError.getDouble(0)/xConst; // x
        adjustments[1] = yError.getDouble(0)/yConst; // y
        adjustments[2] = (DESIRED_AREA - area.getDouble(0))/zConst; // z
        return adjustments;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    /*  This might break the inverse kinematics code, since it may output a coordinate that is outside of the 
    reach of the arm. We might want to implement something inside the inverse kinematics code that makes it so if 
    the returned coordinates are outside the arm's reach, the coordinates are automatically converted to be inside 
    the arm's range.
    */
    @Override
    public void execute() {
        if (joystick.getRawButtonWrapper(ControllerConstants.AIM_ASSIST_BUTTON_NUMBER)) {
            arm.setIntendedCoordinates(arm.getCurrentCoordinates()[0] + getAdjustmentFromError[0], 
                arm.getCurrentCoordinates()[1] + getAdjustmentFromError[1], 
                arm.getCurrentCoordinates()[2] + getAdjustmentFromError[2]);
        } else {
            arm.setIntendedCoordinates(arm.getCurrentCoordinates()[0] + joystick.getHorizontalMovement()*xSpeed, 
                arm.getCurrentCoordinates()[1], + () -> {
                    if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_UP_BUTTON_NUMBER)) {
                        return ySpeed;
                    } else if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_DOWN_BUTTON_NUMBER)) {
                        return -ySpeed;
                    }
                    return 0;
                } 
                arm.getCurrentCoordinates()[2] + joystick.getLateralMovement()*zSpeed);
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
