package frc.robot.commands.clawcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.AbstractJoystick;
import frc.robot.subsystems.ClawGripSubsystem;

public class ClawOpenandCloseCommand extends CommandBase {
    private final ClawGripSubsystem claw;
    private final AbstractJoystick joystick;
    private boolean toggle = true;

    public ClawOpenandCloseCommand(ClawGripSubsystem claw, AbstractJoystick joystick) {
        this.claw = claw;
        this.joystick = joystick;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        // System.out.println("CLAW TOGGLE: " + toggle);
        if(this.joystick.getRawButtonWrapper(ControllerConstants.CLAW_GRIP_OR_RELEASE_BUTTON_NUMBER) && toggle){
            // System.out.println("BUTTON PRESSED");
            if(this.claw.getClawClosed()){
                this.claw.setClawClosed(false);
            }
            else{
                this.claw.setClawClosed(true);
            }
            toggle = false;
        }
        if(this.joystick.getRawButtonReleasedWrapper(ControllerConstants.CLAW_GRIP_OR_RELEASE_BUTTON_NUMBER)){
            // System.out.println("BUTTON RELEASED");
            toggle = true;
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
