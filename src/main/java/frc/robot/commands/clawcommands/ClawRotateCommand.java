package frc.robot.commands.clawcommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ClawRotationSubsystem;
import frc.robot.util.NetworkTablesUtil;

public class ClawRotateCommand extends CommandBase {
    private static final double CLAW_ROTATION_SPEED = 2;

    private final ClawRotationSubsystem claw;
    private final XboxController joystick;

    public ClawRotateCommand(ClawRotationSubsystem claw, XboxController joystick) {
        this.claw = claw;
        this.joystick = joystick;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        if(this.joystick.getRawButtonWrapper(ControllerConstants.AIM_ASSIST_ROTATE_BUTTON_NUMBER)){
            if(NetworkTablesUtil.getLimeLightPipeline() == 1){
                this.claw.autoRotateClaw();
            }
        }
        else{

            int fov = this.joystick.controller.getHID().getPOV();

            if(fov == 90) {
                this.claw.changeAngle(CLAW_ROTATION_SPEED); // clockwise
            } else if(fov == 270) {
                this.claw.changeAngle(-CLAW_ROTATION_SPEED); // counterclockwise
            } else if(fov == 180) {
                this.claw.setTargetAngle(0.0);
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
