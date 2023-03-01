package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A wrapper around {@link CommandXboxController}. Our left joystick is not working right now though, so I'm just going to use the right side one for now.
 */
public class XboxController extends AbstractJoystick {
    private final CommandXboxController controller;

    public XboxController(CommandXboxController controller){
        this.controller = controller;
    }

    public double getRightHorizontalMovement() {
        return controller.getRightX();
    }

    public double getRightLateralMovement() {
        return controller.getRightY();
    }

    public double getLeftHorizontalMovement(){
        return controller.getLeftX();
    }
    
    public double getLeftLateralMovement(){
        return controller.getLeftY();   
    }

    @Override
    public double getHorizontalMovement() {
        return this.getRightHorizontalMovement();
    }

    @Override
    public double getLateralMovement() {
        return this.getRightLateralMovement();
    }

    @Override
    public boolean getRawButtonWrapper(int button) {
        return controller.getHID().getRawButton(button);
    }

    @Override
    public boolean getRawButtonReleasedWrapper(int button) {
        return controller.getHID().getRawButtonReleased(button);
    }
}
