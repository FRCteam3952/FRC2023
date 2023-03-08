package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.util.NetworkTablesUtil;

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
    private static final double EXTEND_RETRACT_SPEED = 0.02; // for possible testing later

    private static final double TURRET_SPEED = 0.5;

    private boolean detectCone = true; // True -> vision is looking for cones, False -> vision is looking for cubes, TODO: implement toggle

    public ArmControlCommand(ArmSubsystem arm, XboxController joystick) {
        this.arm = arm;
        this.joystick = joystick;

        addRequirements(arm);
    }


    // Gets adjustments from limelight and converts them to position adjustments
    private double[] getAdjustmentFromError(boolean flipped) {
        double[] adjustments = new double[3];

        if(flipped){

            double yAdjustment = detectCone ? (DESIRED_AREA_CONE - LimeLight.getArea()) / DESIRED_AREA_CONE : 
                    (DESIRED_AREA_CUBE - LimeLight.getArea()) / DESIRED_AREA_CUBE; // y axis from perspective of the camera
            yAdjustment = yAdjustment > 1 ? 1 : yAdjustment;
    
            adjustments[0] = LimeLight.getYAdjustment(); // x-axis adjustment
    
            adjustments[1] = yAdjustment; // y-axis adjustment
    
            adjustments[2] = LimeLight.getXAdjustment(); // z-axis adjustment
    
        }
        else{

            double xAdjustment = detectCone ? (DESIRED_AREA_CONE - LimeLight.getArea()) / DESIRED_AREA_CONE : 
                    (DESIRED_AREA_CUBE - LimeLight.getArea()) / DESIRED_AREA_CUBE; // z axis from perspective of the camera
            xAdjustment = xAdjustment > 1 ? 1 : xAdjustment;
    
            adjustments[0] = xAdjustment; // x-axis adjustment
    
            adjustments[1] = LimeLight.getYAdjustment(); // y-axis adjustment
    
            adjustments[2] = LimeLight.getXAdjustment(); // z-axis adjustment
    
        }
            
        return adjustments;

    }

    // Primary arm control
    private void primaryArmControl() {

        if(this.arm.getControlMode()){ // only run when arm is in manual control
            if (joystick.getRawButtonWrapper(ControllerConstants.AIM_ASSIST_BUTTON_NUMBER)) { // Aim assist
                arm.setControlDimensions(false);
                double[] adjustments = this.getAdjustmentFromError(this.arm.getFlipped());
                arm.moveVector(-adjustments[0] * X_SPEED, 0, -adjustments[2] * Z_SPEED);
                System.out.println(adjustments[0] * X_SPEED + ", " + adjustments[1] * Y_SPEED + ", " + adjustments[2] * Z_SPEED);
            }
            else{
                this.arm.moveVector(-joystick.getLeftLateralMovement() * X_SPEED, -joystick.getRightLateralMovement() * Y_SPEED, 0);
                this.arm.setTurretSpeed(TURRET_SPEED * (this.joystick.controller.getRightTriggerAxis() - this.joystick.controller.getLeftTriggerAxis()));     
    
                if(this.joystick.getRawButtonPressedWrapper(ControllerConstants.FLIP_ARM_BUTTON_NUMBER)) {
                    this.arm.setFlipped(!this.arm.getFlipped());
                }

                // test later for better understanding, if theres not enough time its okay - max
                double turretAngleRad = this.arm.getCurrentAnglesRad()[2];
                if(this.joystick.getRawButtonPressedWrapper(ControllerConstants.EXTEND_ARM_BUTTON_NUMBER)) {
                    this.arm.moveVector(Math.sin(turretAngleRad) * EXTEND_RETRACT_SPEED, // x
                            0, // y
                            Math.cos(turretAngleRad) * EXTEND_RETRACT_SPEED); // z
                }

                if(this.joystick.getRawButtonPressedWrapper(ControllerConstants.RETRACT_ARM_BUTTON_NUMBER)) {
                    this.arm.moveVector(Math.sin(turretAngleRad) * -EXTEND_RETRACT_SPEED, // x
                            0, // y
                            Math.cos(turretAngleRad) * -EXTEND_RETRACT_SPEED); // z
                }
                
                this.arm.setControlDimensions(true);
            }
            
        }
        if(this.joystick.getRawButtonPressedWrapper(2)){
            if (NetworkTablesUtil.getLimeLightPipeline() == 2){
                NetworkTablesUtil.setLimelightPipeline(1);
            }
            else{
                NetworkTablesUtil.setLimelightPipeline(2);
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
