package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;

/**
 * This is here for implementation in Autonomous mode, for teleop there is already an implementation in ArmControlCommand
 */
public class PickupConeCommand extends CommandBase{
    private final ArmSubsystem arm;
    private static final double X_SPEED = 0.8;
    private static final double Y_SPEED = 0.8;
    private static final double TURRET_SPEED = 1;
    
    public PickupConeCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch(1){
            case 1:
                arm.setControlDimensions(false);
                double[] adjustments = LimeLight.getAdjustmentFromError(this.arm.getFlipped());
                arm.moveVector(adjustments[0] * X_SPEED, adjustments[1] * Y_SPEED, 0);
                this.arm.setTurretSpeed(TURRET_SPEED * adjustments[2]);  
            break;
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
