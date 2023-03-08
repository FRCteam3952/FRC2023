package frc.robot.commands.clawcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ClawRotationSubsystem;

public class ClawRotateCommand extends CommandBase {
    private static final double CORRECT_CLAW_ROTATION_AT_DELTA = 10;
    private static final double CLAW_ROTATION_SPEED = 0.05;

    private final ClawRotationSubsystem claw;
    private final XboxController joystick;

    private boolean wasJustZero = false;
    private double previousRotationsAtZero;

    public ClawRotateCommand(ClawRotationSubsystem claw, XboxController joystick) {
        this.claw = claw;
        this.joystick = joystick;

        addRequirements(claw);
        this.previousRotationsAtZero = this.claw.getClawAngle();
    }

    @Override
    public void execute() {
        // this.claw.setClawRotateSpeed(CLAW_ROTATION_SPEED * (this.joystick.controller.getRightTriggerAxis() - this.joystick.controller.getLeftTriggerAxis()));

        int fov = this.joystick.controller.getHID().getPOV();
        if(fov == 90) {
            this.claw.setClawRotateSpeed(CLAW_ROTATION_SPEED); // clockwise
            this.wasJustZero = true;
        } else if(fov == 270) {
            this.claw.setClawRotateSpeed(-CLAW_ROTATION_SPEED); // counterclockwise
            this.wasJustZero = true;
        } else {
            if(this.wasJustZero) {
                this.previousRotationsAtZero = this.claw.getClawAngle();
                this.wasJustZero = false;
                this.claw.setClawRotateSpeed(0.0);
            } else {
                double difference = this.claw.getClawAngle() - this.previousRotationsAtZero; // if this is positive, we've drifted clockwise. if negative, we've drifted cc-wise (i hope)
                if (Math.abs(difference) > CORRECT_CLAW_ROTATION_AT_DELTA) {
                    this.claw.setClawRotateSpeed(Math.signum(-difference) * CLAW_ROTATION_SPEED);
                } else {
                    this.claw.setClawRotateSpeed(0.0);
                }
            }
        }
        // this.claw.setClawRotateSpeed(this.joystick.getLeftHorizontalMovement() * 0.1);
        /*
        if (this.joystick.getRawButtonWrapper(ControllerConstants.CLAW_ROTATE_RIGHT_BUTTON_NUMBER)) {
            this.claw.setClawRotateSpeed(ClawConstants.CLAW_ROTATE_SPEED);
        } else if (this.joystick.getRawButtonWrapper(ControllerConstants.CLAW_ROTATE_LEFT_BUTTON_NUMBER)) {
            this.claw.setClawRotateSpeed(-ClawConstants.CLAW_ROTATE_SPEED);
        } else {
            this.claw.setClawRotateSpeed(0);
        }
        */
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
