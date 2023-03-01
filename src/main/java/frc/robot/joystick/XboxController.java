package frc.robot.joystick;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A wrapper around {@link CommandXboxController}
 */
public class XboxController {
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

    public boolean getRawButtonWrapper(int button) {
        return controller.getHID().getRawButton(button);
    }
}
