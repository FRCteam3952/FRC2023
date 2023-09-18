package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A wrapper around {@link CommandXboxController} but for the Nintendo Pro Controller.
 * <p>
 * This class is necessary because the Nintendo Pro Controller has different deadzones than the XBox controller, as well as not reaching all values from [-1, 1] on joysticks.
 */
public class NintendoProController extends AbstractJoystick {
    public static final double IGNORE_DELTA = 0.1;

    public final CommandXboxController controller;

    public NintendoProController(CommandXboxController controller) {
        this.controller = controller;
    }

    /**
     * The XBox controller sometimes returns a value between [-0.1, 0.1] when at rest (it's sensitive). To avoid misinputs, this method corrects for that.
     *
     * @param value The value to correct
     * @return 0.0 if the value is within the deadzone {@link #IGNORE_DELTA}, otherwise the value clamped to [-1, 1] to avoid any changed calculations from previous anti-deadzone
     */
    private static double correctDeadzone(double value) {
        if (Math.abs(value) < IGNORE_DELTA) {
            return 0;
        } else {
            return MathUtil.clamp(value, -1d, 1d);
        }
    }

    private double getControllerLeftX() {
        double val = controller.getLeftX();
        if(val < 0) {
            return val / 0.89;
        }
        return val / 0.67;
    }

    private double getControllerLeftY() {
        double val = -controller.getLeftY(); // inverted
        if(val < 0) {
            return val / 0.84;
        }
        return val / 0.74;
    }

    private double getControllerRightX() {
        double val = controller.getRightX();
        if(val < 0) {
            return val / 0.76;
        }
        return val / 0.74;
    }

    private double getControllerRightY() {
        double val = -controller.getRightY();
        if(val < 0) {
            return val / 0.76;
        }
        return val / 0.8;
    }

    public double getRightHorizontalMovement() {
        return correctDeadzone(this.getControllerRightX());
    }

    public double getRightVerticalMovement() {
        return correctDeadzone(this.getControllerRightY());
    }

    public double getLeftHorizontalMovement() {
        return correctDeadzone(this.getControllerLeftX());
    }

    public double getLeftVerticalMovement() {
        return correctDeadzone(this.getControllerLeftY());
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
