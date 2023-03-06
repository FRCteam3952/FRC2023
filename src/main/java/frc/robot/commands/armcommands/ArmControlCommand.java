package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;

/**
 * Moves arm on the turret
 */
public class ArmControlCommand extends CommandBase {
    private static final double DESIRED_AREA_CONE = 5000; // tentative measurement, pixels
    private static final double DESIRED_AREA_CUBE = 420; // measure later

    private final ArmSubsystem arm;
    private final XboxController joystick;

    // Inches per 20ms
    private static final double X_SPEED = 0.3;
    private static final double Y_SPEED = 0.3;
    private static final double Z_SPEED = 0.3;

    private static final double TURRET_SPEED = 0.2;

    private boolean detectCone = true; // True -> vision is looking for cones, False -> vision is looking for cubes, TODO: implement toggle

    public ArmControlCommand(ArmSubsystem arm, XboxController joystick) {
        this.arm = arm;
        this.joystick = joystick;

        addRequirements(arm);
    }


    // Gets adjustments from limelight and converts them to position adjustments
    private double[] getAdjustmentFromError() {
        double[] adjustments = new double[4];
        double turretAngle = arm.getCurrentAnglesRad()[2];
        double zAdjustment = detectCone ? (DESIRED_AREA_CONE - LimeLight.getArea()) / DESIRED_AREA_CONE : 
                (DESIRED_AREA_CUBE - LimeLight.getArea()) / DESIRED_AREA_CUBE; // z axis from perspective of the camera
        zAdjustment = zAdjustment > 1 ? 1 : zAdjustment;

        adjustments[0] = Math.sin(turretAngle) * zAdjustment; // x-axis adjustment

        adjustments[1] = LimeLight.getYAdjustment(); // y-axis adjustment

        adjustments[2] = Math.cos(turretAngle) * zAdjustment; // z-axis adjustment

        adjustments[3] = LimeLight.getXAdjustment(); // turret angle adjustment

        return adjustments;
    }

    // Primary arm control
    private void primaryArmControl() {

        if(this.arm.getControlMode()){ // only run when arm is in manual control
            if (joystick.getRawButtonWrapper(ControllerConstants.AIM_ASSIST_BUTTON_NUMBER)) { // Aim assist
                arm.setControlDimensions(false);
                double[] adjustments = this.getAdjustmentFromError();
                //arm.moveVector(adjustments[0] * X_SPEED, adjustments[1] * Y_SPEED, adjustments[2] * Z_SPEED);
                System.out.println(adjustments[3] * TURRET_SPEED + ", " + adjustments[0] * X_SPEED + ", " + adjustments[1] * Y_SPEED + ", " + adjustments[2] * Z_SPEED);
            }
            else{
                this.arm.moveVector(-joystick.getLeftLateralMovement() * Z_SPEED, -joystick.getRightLateralMovement() * Y_SPEED, 0);
                this.arm.setTurretSpeed(X_SPEED * (this.joystick.controller.getRightTriggerAxis() - this.joystick.controller.getLeftTriggerAxis()));     
    
                if(this.joystick.getRawButtonPressedWrapper(ControllerConstants.FLIP_ARM_BUTTON_NUMBER)) {
                    this.arm.setFlipped(!this.arm.getFlipped());
                }
    
                this.arm.setControlDimensions(true);
            }
        }
    
        if(this.joystick.getRawButtonPressedWrapper(4)){
            this.arm.setPIDControlState(!this.arm.getPIDControlOn());
        }
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        primaryArmControl();
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
