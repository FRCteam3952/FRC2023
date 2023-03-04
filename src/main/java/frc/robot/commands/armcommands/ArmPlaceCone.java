package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawGripSubsystem;

import frc.robot.Constants.PositionConstants;

public class ArmPlaceCone extends CommandBase {
    // private static final double CLAW_ROTATION_SPEED = 0.05;

    // private final ClawRotationSubsystem claw;
    // private final XboxController joystick;

    private final ArmSubsystem arm;
    private final ClawGripSubsystem claw;

    public ArmPlaceCone(ArmSubsystem arm, ClawGripSubsystem claw) {
        this.arm = arm;
        this.claw = claw;
        
        addRequirements(arm, claw);
    }
    
    @Override
    public void execute() {
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
