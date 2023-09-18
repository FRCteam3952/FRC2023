package frc.robot.commands.armcommands;

import edu.wpi.first.math.MathUtil;
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

    private final ArmSubsystem arm;
    private final XboxController controller;

    // Inches per 20ms
    private static final double X_SPEED = 2;
    private static final double Y_SPEED = 2;
    private static final double TURRET_SPEED = 0.25;
    private static double turret_adjust = 0.0;

    public ArmControlCommand(ArmSubsystem arm, XboxController controller) {
        this.arm = arm;
        this.controller = controller;

        addRequirements(arm);
    }

    /*
     * Primary arm control
     */
    private void primaryArmControl() {

        if (this.arm.getControlMode()) { // only run when arm is in manual control
            this.arm.setis2D(true);
            armAimAssist();

            int mult = this.arm.isAtHumanPlayer() ? -1 : 1;
            this.arm.setTurretSpeed(mult * -MathUtil.clamp(controller.getLeftHorizontalMovement() * TURRET_SPEED + turret_adjust, -1, 1));
            this.arm.moveVector(controller.getLeftVerticalMovement() * X_SPEED * mult, -controller.getRightVerticalMovement() * Y_SPEED, 0);

        }
        else{
            this.arm.setis2D(false);
        }
        if (this.controller.getRawButtonPressedWrapper(ControllerConstants.TOGGLE_PID_BUTTON_NUMBER)) { //toggle PID off
            this.arm.setPIDControlState(false);
        }

    }

    /*
     * Handles Limelight aim assist for arm
     */
    private void armAimAssist() {
        boolean rightTrigger = this.controller.controller.getRightTriggerAxis() > 0.2, leftTrigger = this.controller.controller.getLeftTriggerAxis() > 0.2;
        if (rightTrigger && leftTrigger) {
            NetworkTablesUtil.setLimelightPipeline(4);
        } else if (rightTrigger) { // cone PID, if > 0.9 do rotation as well but we don't do that here (look in ClawRotateCommand)
            NetworkTablesUtil.setLimelightPipeline(1);
        } else if (leftTrigger) { // just cube PID
            NetworkTablesUtil.setLimelightPipeline(3);
        }
        if (rightTrigger || leftTrigger) {
            if (!this.arm.getFlipped()) {
                double[] adjustments = LimeLight.getAdjustmentFromError(this.arm.getFlipped());
                //arm.moveVector(adjustments[0] * X_SPEED, adjustments[1] * Y_SPEED, 0);
                turret_adjust = adjustments[2];
            }
        } else {
            turret_adjust = 0.0;
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
