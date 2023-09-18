package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * A wrapper around {@link CommandJoystick}, contains
 * simple code to handle dead-zones, cut out joystick noise, and to provide button inputs for flight joystick.
 * <p>
 * The values here were copied from previous years' code and is known to work, so don't change them.
 */
public class FlightJoystick extends AbstractJoystick {
    public static final double c = 0.1;
    public static final double deadzone = 0.2;
    public static final double max = 0.8;
    public static final double k = (max - c) / Math.log(2 - deadzone);

    public static final double cT = 0.08;
    public static final double deadzoneT = 0.08;
    public static final double maxT = 0.4;
    public static final double kT = (maxT - cT) / Math.log(2 - deadzoneT);

    public CommandJoystick joystick;

    public FlightJoystick(CommandJoystick joystick) {
        this.joystick = joystick;
    }

    @Override
    public double getHorizontalMovement() {
        double x = joystick.getX();

        return Math.abs(x) >= deadzone ? k * Math.signum(x) * (Math.log(Math.abs(x) + 1 - deadzone) + c) : 0;
    }

    @Override
    public double getVerticalMovement() {
        double y = joystick.getY();

        return Math.abs(y) >= deadzone ? k * Math.signum(y) * (Math.log(Math.abs(y) + 1 - deadzone) + c) : 0;
    }

    public double getRotation() {
        double t = joystick.getZ();

        return Math.abs(t) >= deadzoneT ? kT * Math.signum(t) * (Math.log(Math.abs(t) + 1 - deadzoneT) + cT) : 0;
    }

    @Override
    public boolean getRawButtonWrapper(int button) {
        return this.joystick.getHID().getRawButton(button);
    }

    @Override
    public boolean getRawButtonReleasedWrapper(int button) {
        return this.joystick.getHID().getRawButtonReleased(button);
    }

}
