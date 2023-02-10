package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Moves arm on the turret
 */
public class ArmControlCommand extends CommandBase{
    private final ArmSubsystem arm;
    private final FlightJoystick joystick;

    public ArmControlCommand(ArmSubsystem arm, FlightJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }


    // Primary arm control
    private void primaryArmControl() {
        
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        primaryArmControl();
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
