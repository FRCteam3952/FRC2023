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
public class ArmControlCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final FlightJoystick joystick;
    private final double areaConst;
    private final double xSpeed, ySpeed, zSpeed, turretSpeed;
    private static final double DESIRED_AREA = 369; // in pixels probably, can tune later

    public ArmControlCommand(ArmSubsystem arm, FlightJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
        this.areaConst = 60; //can tune later
        this.xSpeed = 0.3; // Inches per 20ms
        this.ySpeed = 0.3; // Inches per 20ms
        this.zSpeed = 0.3; // Inches per 20ms
        this.turretSpeed = 0.2;

        addRequirements(arm);
    }


    // Gets adjustments from limelight and converts them to position adjustments
    private double[] getAdjustmentFromError() {
        double[] adjustments = new double[4];
        adjustments[0] = Math.sin(arm.getCurrentAnglesRad()[2]) * (DESIRED_AREA - LimeLightSubsystem.getArea()) / areaConst; // x-axis adjustment

        adjustments[1] = LimeLightSubsystem.getYAdjustment(); // y-axis adjustment

        adjustments[2] = Math.cos(arm.getCurrentAnglesRad()[2]) * (DESIRED_AREA - LimeLightSubsystem.getArea() / areaConst); // z-axis adjustment

        adjustments[3] = LimeLightSubsystem.getXAdjustment(); // turret angle adjustment

        return adjustments;
    }

    // Primary arm control
    private void primaryArmControl() {
        if (joystick.getRawButtonWrapper(ControllerConstants.AIM_ASSIST_BUTTON_NUMBER)) { // Aim assist
            double[] adjustments = this.getAdjustmentFromError();
            arm.setTurretSpeed(adjustments[3] * turretSpeed);
            arm.moveVector(adjustments[0] * xSpeed, adjustments[1] * ySpeed, adjustments[2] * zSpeed);
        } else {
            double y = 0;
            if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_UP_BUTTON_NUMBER)) {
                y = ySpeed;
            } else if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_DOWN_BUTTON_NUMBER)) {
                y = -ySpeed;
            }
            arm.setTurretSpeed(joystick.getHorizontalMovement() * turretSpeed);
            double turretAngleRad = arm.getCurrentAnglesRad()[2];
            arm.moveVector(Math.sin(turretAngleRad) * joystick.getLateralMovement() * xSpeed, // Handles extension of robot arm 
                    y,
                    Math.cos(turretAngleRad) * joystick.getLateralMovement() * zSpeed);
        }
    }

    // Moves arm to preset distance above the floor for picking up gamepieces 
    private void pickUpPosition() {
        if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_TO_PICK_UP_POSITION_BUTTON_NUMBER)) {
            double[] armCoordinates = arm.getCurrentCoordinates();
            arm.setIntendedCoordinates(armCoordinates[0], ArmConstants.PICK_UP_POSITION_Y, armCoordinates[2]);
        }
    }


    // Test primary arm control
    private void testPrimaryArmControl() {
        arm.moveVector(joystick.getLateralMovement() * xSpeed, 0, 0);
    }

    // Toggles whether PID control is active or not
    private void togglePIDControl() {
        if (joystick.getRawButtonWrapper(ControllerConstants.PID_CONTROL_TOGGLE_BUTTON_NUMBER)) {
            arm.setPIDControlOn(!arm.getPIDControlOn());
        }
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setPIDControlOn(false);
    }

    @Override
    public void execute() {
        // testPrimaryArmControl();
        primaryArmControl();
        // pickUpPosition();
        togglePIDControl();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
