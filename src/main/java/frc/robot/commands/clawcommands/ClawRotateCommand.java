package frc.robot.commands.clawcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.FlightJoystick;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawRotationSubsystem;

public class ClawRotateCommand extends CommandBase {
    private final ClawRotationSubsystem claw;
    private final FlightJoystick joystick;

    public ClawRotateCommand(ClawRotationSubsystem claw, FlightJoystick joystick) {
        this.claw = claw;
        this.joystick = joystick;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        if (this.joystick.getRawButtonWrapper(ControllerConstants.CLAW_ROTATE_RIGHT_BUTTON_NUMBER)) {
            this.claw.setClawRotateSpeed(ClawConstants.CLAW_ROTATE_SPEED);
        } else if (this.joystick.getRawButtonWrapper(ControllerConstants.CLAW_ROTATE_LEFT_BUTTON_NUMBER)) {
            this.claw.setClawRotateSpeed(-ClawConstants.CLAW_ROTATE_SPEED);
        } else {
            this.claw.setClawRotateSpeed(0);
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
