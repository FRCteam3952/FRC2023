package frc.robot.commands;
import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.ArmInverseKinematicsConstants;

public class ArmControlCommand extends CommandBase{
    private final ArmSubsystem arm;
    private final FlightJoystick joystick;
    public ArmControlCommand(ArmSubsystem arm, FlightJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setIntendedCoordinates(joystick.getHorizontalMovement(), ArmInverseKinematicsConstants.ORIGIN_HEIGHT, joystick.getLateralMovement());
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
