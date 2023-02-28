package frc.robot.joystick;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxController{
    public CommandXboxController controller;

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
}
