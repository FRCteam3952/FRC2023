package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;

/**
 * Moves arm on the turret
 */
public class ArmControlCommand extends CommandBase {
    private static final double DESIRED_AREA = 369; // in pixels probably, can tune later

    private final ArmSubsystem arm;
    private final XboxController joystick;
    private static final double AREA_CONST = 60; // can tune later

    // inches per 20ms
    private static final double X_SPEED = 0.3;
    private static final double Y_SPEED = 0.3;
    private static final double Z_SPEED = 0.3;

    private static final double TURRET_SPEED = 0.2;

    public ArmControlCommand(ArmSubsystem arm, XboxController joystick) {
        this.arm = arm;
        this.joystick = joystick;

        addRequirements(arm);
    }


    // Gets adjustments from limelight and converts them to position adjustments
    private double[] getAdjustmentFromError() {
        double[] adjustments = new double[4];
        double turretAngle = arm.getCurrentAnglesRad()[2];
        double zAdjustment = (DESIRED_AREA - LimeLight.getArea()) / AREA_CONST; // z axis from perspective of the camera

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
            arm.setTurretSpeed(adjustments[3] * TURRET_SPEED);
            arm.moveVector(adjustments[0] * X_SPEED, adjustments[1] * Y_SPEED, adjustments[2] * Z_SPEED);
        } else {
            double y = 0;
            if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_UP_BUTTON_NUMBER)) {
                y = Y_SPEED;
            } else if (joystick.getRawButtonWrapper(ControllerConstants.MOVE_ARM_DOWN_BUTTON_NUMBER)) {
                y = -Y_SPEED;
            }
            arm.moveVector(joystick.getLateralMovement() * X_SPEED, y, joystick.getHorizontalMovement() * Z_SPEED);
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

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        primaryArmControl();
        pickUpPositionFlipped();
        pickUpPositionNotFlipped();
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
