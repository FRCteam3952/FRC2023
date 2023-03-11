package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.util.NetworkTablesUtil;

/**
 * Moves arm on the turret
 */
public class ArmControlCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final XboxController joystick;

    // Inches per 20ms
    private static final double X_SPEED = 0.9;
    private static final double Y_SPEED = 0.9;
    private static final double Z_SPEED = 0.9;
    private static final double EXTEND_RETRACT_SPEED = 0.02; // for possible testing later
    private static double turret_adjust = 0.0;

    private static final double TURRET_SPEED = 0.2;

    public ArmControlCommand(ArmSubsystem arm, XboxController joystick) {
        this.arm = arm;
        this.joystick = joystick;

        addRequirements(arm);
    }

    // Assumes goTowardsIntendedCoordinates() is running and PID is on
    private void setYPosition() {
        if (joystick.getRawButtonPressedWrapper(ControllerConstants.HUMAN_STATION_HEIGHT_BUTTON_NUMBER)) {
            double[] currentCoords = arm.getTargetCoordinates();
            (new GoTowardsCoordinatesCommandTeleop(this.arm,(new double[]{currentCoords[0],ArmConstants.HUMAN_PLAYER_HEIGHT,currentCoords[2]}),this.joystick,0.4,0.4)).schedule();
    
        } else if (joystick.getRawButtonPressedWrapper(ControllerConstants.PICK_UP_HEIGHT_BUTTON_NUMBER)) {
            double[] currentCoords = arm.getTargetCoordinates();
            (new GoTowardsCoordinatesCommandTeleop(this.arm,(new double[]{currentCoords[0],ArmConstants.PICK_UP_POSITION_Y,currentCoords[2]}),this.joystick,0.2,0.4)).schedule();
        }
    }

    // Primary arm control
    private void primaryArmControl() {
        
        if(this.arm.getControlMode()){ // only run when arm is in manual control
            boolean rightTrigger = this.joystick.controller.getRightTriggerAxis() > 0.2, leftTrigger = this.joystick.controller.getLeftTriggerAxis() > 0.2;

            if(rightTrigger && leftTrigger) {
                NetworkTablesUtil.setLimelightPipeline(4);
            } else if(rightTrigger) { // cone PID, if > 0.9 do rotation as well but we don't do that here (look in ClawRotateCommand)
                NetworkTablesUtil.setLimelightPipeline(1);
            } else if(leftTrigger) { // just cube PID
                NetworkTablesUtil.setLimelightPipeline(3);
            }

            if(rightTrigger || leftTrigger) {
                if(!this.arm.getFlipped()){
                    arm.setControlDimensions(false);
                    double[] adjustments = LimeLight.getAdjustmentFromError(this.arm.getFlipped());
                    //arm.moveVector(adjustments[0] * X_SPEED, adjustments[1] * Y_SPEED, 0);
                    turret_adjust = adjustments[2];
                } else {
                    System.out.println("yo mama");
                }
            } else {
                turret_adjust = 0;
            }

            this.arm.moveVector(-joystick.getLeftLateralMovement() * X_SPEED, -joystick.getRightLateralMovement() * Y_SPEED, 0);
            this.arm.setTurretSpeed(joystick.getLeftHorizontalMovement() * TURRET_SPEED + turret_adjust);
            // this.arm.setTurretSpeed(TURRET_SPEED * (this.joystick.controller.getRightTriggerAxis() - this.joystick.controller.getLeftTriggerAxis()) + turret_adjust);     

            if(this.joystick.getRawButtonPressedWrapper(ControllerConstants.FLIP_ARM_BUTTON_NUMBER)) {
                // this.arm.setFlipped(!this.arm.getFlipped());
            }

            /*
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
            } */
            
            this.arm.setControlDimensions(true);
            
        }

        /*
        if(this.joystick.getRawButtonPressedWrapper(ControllerConstants.TOGGLE_PIPELINES)){
            if (NetworkTablesUtil.getLimeLightPipeline() == 3){
                NetworkTablesUtil.setLimelightPipeline(1);
            }
            else{
                NetworkTablesUtil.setLimelightPipeline(3);
            }
            
        }
        if(this.joystick.getRawButtonPressedWrapper(3)){
            if(NetworkTablesUtil.getLimeLightPipeline() != 4){
                NetworkTablesUtil.setLimelightPipeline(4);
            }
            else{
                NetworkTablesUtil.setLimelightPipeline(1);
            }

        } */
        if(this.joystick.getRawButtonPressedWrapper(ControllerConstants.TOGGLE_PID_BUTTON_NUMBER)){
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
        setYPosition();

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
