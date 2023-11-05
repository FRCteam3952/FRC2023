package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A wrapper around {@link CommandXboxController} but for the Nintendo Pro Controller.
 * <p>
 * This class is necessary because the Nintendo Pro Controller has different deadzones than the XBox controller, as well as not reaching all values from [-1, 1] on joysticks.
 */
public class NintendoProController extends AbstractJoystick {
    public static final double IGNORE_DELTA = 0.15;

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
        return value;
        /*
        if (Math.abs(value) < IGNORE_DELTA) {
            return 0;
        } else {
            return MathUtil.clamp(value, -1d, 1d);
        }*/
    }

    /**
     * Applies a deadzone and scale to the raw controller values.
     * @param rawVal The raw value from the controller
     * @param maxValPos The maximum (farthest from 0) value possible for rawVal > 0. This value should be POSITIVE.
     * @param maxValNeg The maximum (farthest from 0) value possible for rawVal < 0. This value should be NEGATIVE.
     * @return
     */
    private double deadzoneRawVals(double rawVal, double maxValPos, double maxValNeg) {
        double absVal = Math.abs(rawVal);
        if(absVal < 0.15) {
            return 0;
        }
        if(rawVal > 0) {
            return (absVal - IGNORE_DELTA) / (maxValPos - 0.15);
        }
        return (absVal - IGNORE_DELTA) / (maxValNeg + 0.15);
    }

    private double getControllerLeftX() {
        return deadzoneRawVals(controller.getRawAxis(0), 0.67, -0.89);
        /*
        if(val < 0) {
            return val / deadzoneRawVals(0.89);
        }
        return val / deadzoneRawVals(0.67);*/
    }

    private double getControllerLeftY() {
        return deadzoneRawVals(-controller.getRawAxis(1), 0.74, -0.84); // inverted
        /*
        if(val < 0) {
            return val / deadzoneRawVals(0.84);
        }
        return val / deadzoneRawVals(0.74);*/
    }

    private double getControllerRightX() {
        return deadzoneRawVals(controller.getRawAxis(2), 0.74, -0.76);
        /*
        if(val < 0) {
            return val / deadzoneRawVals(0.76);
        }
        return val / 0.74;*/
    }

    private double getControllerRightY() {
        return deadzoneRawVals(-controller.getRawAxis(3), 0.8, -0.76); // inverted
        /*
        if(val < 0) {
            return val / deadzoneRawVals(0.76);
        }
        return val / deadzoneRawVals(0.8); */
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
