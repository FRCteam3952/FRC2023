package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

/**
 * Moves arm on the turret
 */
public class ArmControlCommand extends CommandBase{
    private final ArmSubsystem arm;
    private final FlightJoystick joystick;
    private final double areaConst;
    private final double xSpeed, ySpeed, zSpeed;
    private final double DESIRED_AREA = 369; // in pixels probably, can tune later

    public ArmControlCommand(ArmSubsystem arm, FlightJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
        this.areaConst = 60; //can tune later
        this.xSpeed = 0.1; // Inches per 20ms
        this.ySpeed = 0.05; // Inches per 20ms
        this.zSpeed = 0.1; // Inches per 20ms

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Gets adjustments from limelight and converts them to position adjustments
    private double[] getAdjustmentFromError() {
        double[] adjustments = new double[3];
        adjustments[0] = Math.sin(arm.getCurrentAnglesRad()[2]) * LimeLightSubsystem.getXAdjustment() 
            + Math.cos(arm.getCurrentAnglesRad()[2]) * (DESIRED_AREA - LimeLightSubsystem.getArea())/areaConst; // x-axis adjustment

        adjustments[1] = LimeLightSubsystem.getYAdjustment(); // y-axis adjustment

        adjustments[2] = Math.cos(arm.getCurrentAnglesRad()[2]) * LimeLightSubsystem.getXAdjustment() 
            + Math.sin(arm.getCurrentAnglesRad()[2]) * (DESIRED_AREA - LimeLightSubsystem.getArea()/areaConst); // z-axis adjustment

        return adjustments;
}

// Primary arm control with PID
private void primaryArmControlPID() {
    if (joystick.getRawButtonWrapper(ControllerConstants.AIM_ASSIST_BUTTON_NUMBER)) {
        arm.moveByElement(getAdjustmentFromError()[0] * xSpeed, getAdjustmentFromError()[1] * ySpeed, getAdjustmentFromError()[2] * zSpeed);
    } else {
        double y = 0;
        if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_UP_BUTTON_NUMBER)) {
            y =  ySpeed;
        } else if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_DOWN_BUTTON_NUMBER)) {
            y = -ySpeed;
        } 
        arm.moveByElement(joystick.getHorizontalMovement() * xSpeed, y, joystick.getLateralMovement() * zSpeed);
    }
}

// Primary arm control with direct powering of motors
// Work in progress?
private void primaryArmControlDirect() {
    arm.movePolar(joystick.getLateralMovement(), arm.getCurrentAnglesDeg()[2]);
}

// Moves arm to preset distance above the floor for picking up gamepieces 
private void pickUpPosition() {
    if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_TO_PICK_UP_POSITION_BUTTON_NUMBER)) {
        arm.setIntendedCoordinates(arm.getCurrentCoordinates()[0], ArmConstants.PICK_UP_POSITION_Y, arm.getCurrentCoordinates()[2]);
    }
}

// Called when the command is initially scheduled.
@Override
public void initialize() {}


// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {

    // primaryArmControlPID();
    primaryArmControlDirect();
    pickUpPosition();
            
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
