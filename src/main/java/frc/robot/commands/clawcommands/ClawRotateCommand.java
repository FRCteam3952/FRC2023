package frc.robot.commands.clawcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ClawRotationSubsystem;

public class ClawRotateCommand extends CommandBase {
    private static final double CLAW_ROTATION_SPEED = 0.05;

    private final ClawRotationSubsystem claw;
    private final XboxController joystick;

    public ClawRotateCommand(ClawRotationSubsystem claw, XboxController joystick) {
        this.claw = claw;
        this.joystick = joystick;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        int fov = this.joystick.controller.getHID().getPOV();
        if(fov == 90) {
            this.claw.setClawRotateSpeed(CLAW_ROTATION_SPEED);
        } else if(fov == 270) {
            this.claw.setClawRotateSpeed(-CLAW_ROTATION_SPEED);
        } else {
            this.claw.setClawRotateSpeed(0.0);
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
