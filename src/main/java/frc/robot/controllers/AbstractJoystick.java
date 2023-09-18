package frc.robot.controllers;


/**
 * An abstract class for the joystick wrappers. Contains the common methods that we wrap around.
 */
public abstract class AbstractJoystick {

    public abstract double getHorizontalMovement();

    public abstract double getVerticalMovement();

    public abstract boolean getRawButtonWrapper(int button);

    public abstract boolean getRawButtonReleasedWrapper(int button);

}
