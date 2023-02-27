package frc.robot.commands.clawcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
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
        if(this.joystick.getRawButtonWrapper(ControllerConstants.CLAW_GRIP_OR_RELEASE_BUTTON_NUMBER)){
            if(this.claw.getClawClosed()){
                this.claw.setForward();
                this.claw.setClawClosed(false);
            }
            else{
                this.claw.setReverse();
                this.claw.setClawClosed(true);
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
