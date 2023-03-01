package frc.robot.commands.armcommands;

<<<<<<< HEAD
import frc.robot.joystick.AbstractJoystick;
=======
import frc.robot.controllers.FlightJoystick;
>>>>>>> 1c2c614711b9fe8d874f161d469233dc16814497
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmTestCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final AbstractJoystick joystick;

    public ArmTestCommand(ArmSubsystem arm, AbstractJoystick joystick) {
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
        arm.setPivot1Speed(joystick.getLateralMovement() / 2);
        arm.setPivot2Speed(joystick.getHorizontalMovement() / 2);
        if (joystick.getRawButtonWrapper(8)) {
            arm.setTurretSpeed(-0.3);
        } else if (joystick.getRawButtonWrapper(9)) {
            arm.setTurretSpeed(0.3);
        } else {
            arm.setTurretSpeed(0);
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
