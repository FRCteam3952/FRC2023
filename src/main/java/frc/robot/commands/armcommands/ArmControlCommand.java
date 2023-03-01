package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.FlightJoystick;
import frc.robot.Constants.ArmConstants;
import frc.robot.joystick.AbstractJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;

/**
 * Moves arm on the turret
 */
public class ArmControlCommand extends CommandBase {
    private static final double DESIRED_AREA = 369; // in pixels probably, can tune later

    private final ArmSubsystem arm;
    private final AbstractJoystick joystick;
    private final double areaConst;
    private final double xSpeed, ySpeed, zSpeed, turretSpeed;

    public ArmControlCommand(ArmSubsystem arm, AbstractJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
        this.areaConst = 60; // can tune later
        this.xSpeed = 0.3; // Inches per 20ms
        this.ySpeed = 0.3; // Inches per 20ms
        this.zSpeed = 0.3; // Inches per 20ms
        this.turretSpeed = 0.2;

        addRequirements(arm);
    }


    // Gets adjustments from limelight and converts them to position adjustments
    private double[] getAdjustmentFromError() {
        double[] adjustments = new double[4];
        double turretAngle = arm.getCurrentAnglesRad()[2];
        double zAdjustment = (DESIRED_AREA - LimeLight.getArea()) / areaConst; // z axis from perspective of the camera

        adjustments[0] = Math.sin(turretAngle) * zAdjustment; // x-axis adjustment

        adjustments[1] = LimeLight.getYAdjustment(); // y-axis adjustment

        adjustments[2] = Math.cos(turretAngle) * zAdjustment; // z-axis adjustment

        adjustments[3] = LimeLight.getXAdjustment(); // turret angle adjustment

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
            double turretAngle = arm.getCurrentAnglesRad()[2];
            arm.setTurretSpeed(joystick.getHorizontalMovement() * turretSpeed);
            arm.moveVector(Math.sin(turretAngle) * joystick.getLateralMovement() * xSpeed, // Handles extension of robot arm 
                    y, Math.cos(turretAngle) * joystick.getLateralMovement() * zSpeed);
        }
    }

    // Moves arm to preset distance above the floor for picking up gamepieces 
    private void pickUpPositionFlipped() {
        if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_TO_PICK_UP_POSITION_BUTTON_NUMBER_FLIPPED)) {
            arm.setIntendedCoordinates(arm.getCurrentCoordinates()[0], ArmConstants.PICK_UP_POSITION_Y, arm.getCurrentCoordinates()[2], true);
        }
    }

    private void pickUpPositionNotFlipped() {
        if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_TO_PICK_UP_POSITION_BUTTON_NUMBER_NOT_FLIPPED)) {
            arm.setIntendedCoordinates(arm.getCurrentCoordinates()[0], ArmConstants.PICK_UP_POSITION_Y, arm.getCurrentCoordinates()[2], false);
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
        pickUpPositionFlipped();
        pickUpPositionNotFlipped();
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
