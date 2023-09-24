package frc.robot.commands.clawcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.NintendoProController;
import frc.robot.subsystems.ClawRotationSubsystem;

public class ClawRotateCommand extends CommandBase {
    private static final double CLAW_ROTATION_SPEED = 4;

    private final ClawRotationSubsystem claw;
    private final NintendoProController joystick;

    public ClawRotateCommand(ClawRotationSubsystem claw, NintendoProController joystick) {
        this.claw = claw;
        this.joystick = joystick;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        if (this.joystick.controller.getRightTriggerAxis() > 0.9 && !(this.joystick.controller.getLeftTriggerAxis() > 0.2)) { // if > 0.9 we do PID and also rotate
            // this.claw.autoRotateClaw();
        } else {
            int fov = this.joystick.controller.getHID().getPOV();

            if (fov == 0) {
                this.claw.changeAngle(CLAW_ROTATION_SPEED); // clockwise
            } else if (fov == 180) {
                this.claw.changeAngle(-CLAW_ROTATION_SPEED); // counterclockwise
            }
        }

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
