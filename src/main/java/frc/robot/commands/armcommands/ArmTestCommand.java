package frc.robot.commands.armcommands;

import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmTestCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final XboxController joystick;

    public ArmTestCommand(ArmSubsystem arm, XboxController joystick) {
        this.arm = arm;
        this.joystick = joystick;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setPivot1Speed(-joystick.getRightLateralMovement() / 2);
        arm.setPivot2Speed(joystick.getLeftLateralMovement() / 2);

        arm.setTurretSpeed((joystick.controller.getLeftTriggerAxis() - joystick.controller.getRightTriggerAxis()) * 0.5);

        if(this.joystick.getRawButtonPressedWrapper(ControllerConstants.TOGGLE_PID_BUTTON_NUMBER)){
            this.arm.setPIDControlState(true);
        }
        /*
        if (joystick.getRawButtonWrapper(5)) { // THIS IS THE XBOX VALUE. Original FlightJoystick 8
            arm.setTurretSpeed(0.3);
        } else if (joystick.getRawButtonWrapper(6)) { // THIS IS THE XBOX VALUE. Original FlightJoystick 9
            arm.setTurretSpeed(-0.3);
        } else {
            arm.setTurretSpeed(0);
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
