package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A wrapper around {@link CommandXboxController}. Our left joystick is not working right now though, so I'm just going to use the right side one for now.
 */
public class XboxController extends AbstractJoystick {
    public static final double IGNORE_DELTA = 0.08;

    public final CommandXboxController controller;

    public XboxController(CommandXboxController controller) {
        this.controller = controller;
    }

    /**
     * The XBox controller sometimes returns a value of 0.01 to 0.03 when at rest. To avoid floating point errors, this method corrects for that.
     *
     * @param value The value to correct
     * @return 0.0 if the value is within the deadzone {@link #IGNORE_DELTA}, otherwise the value
     */
    private static double correctDeadzone(double value) {
        if (Math.abs(value) < IGNORE_DELTA) {
            return 0;
        } else {
            return value;
        }
    }

    public double getRightHorizontalMovement() {
        return correctDeadzone(controller.getRightX());
    }

    public double getRightVerticalMovement() {
        return correctDeadzone(controller.getRightY());
    }

    public double getLeftHorizontalMovement() {
        return correctDeadzone(controller.getLeftX());
    }

    public double getLeftVerticalMovement() {
        return correctDeadzone(controller.getLeftY());
    }

    @Override
    public double getHorizontalMovement() {
        return this.getRightHorizontalMovement();
    }

    @Override
    public double getVerticalMovement() {
        return this.getRightVerticalMovement();
    }

    @Override
    public boolean getRawButtonWrapper(int button) {
        return controller.getHID().getRawButton(button);
    }

    @Override
    public boolean getRawButtonReleasedWrapper(int button) {
        return controller.getHID().getRawButtonReleased(button);
    }

    public boolean getRawButtonPressedWrapper(int button) {
        return controller.getHID().getRawButtonPressed(button);
    }
}
