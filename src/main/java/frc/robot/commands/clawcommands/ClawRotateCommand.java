package frc.robot.commands.clawcommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ClawRotationSubsystem;

public class ClawRotateCommand extends CommandBase {
    private static final double CLAW_ROTATION_SPEED = 0.5;

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
            this.claw.changeAngle(CLAW_ROTATION_SPEED); // clockwise
        } else if(fov == 270) {
            this.claw.changeAngle(-CLAW_ROTATION_SPEED); // counterclockwise
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
