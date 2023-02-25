package frc.robot.commands.clawcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.ClawGripSubsystem;

public class ClawOpenandCloseCommand extends CommandBase {
    private final ClawGripSubsystem claw;
    private final FlightJoystick joystick;

    public ClawOpenandCloseCommand(ClawGripSubsystem claw, FlightJoystick joystick) {
        this.claw = claw;
        this.joystick = joystick;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        if(this.joystick.getRawButtonWrapper(ControllerConstants.CLAW_GRIP_BUTTON_NUMBER)) {
            this.claw.setSpeed(ClawConstants.CLAW_GRIP_SPEED);
            this.claw.setClawClosed(true);
        } else if(this.joystick.getRawButtonWrapper(ControllerConstants.CLAW_RELEASE_BUTTON_NUMBER)) {
            this.claw.setSpeed(-ClawConstants.CLAW_GRIP_SPEED);
            this.claw.setClawClosed(false);
        } else {
            if(this.claw.getClawClosed()) {
                this.claw.setSpeed(ClawConstants.CLAW_GRIP_SPEED_PASSIVE);
            } else {
                this.claw.setSpeed(0);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
