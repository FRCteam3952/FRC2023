package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

import frc.robot.Constants.ClawConstants;

public class ClawRotateCommand extends CommandBase{
    private final ClawSubsystem clawSubsystem;
    private double angle;

    public ClawRotateCommand(ClawSubsystem clawSubsystem, double angle){
        this.clawSubsystem = clawSubsystem;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute(){
        if(clawSubsystem.getClawAngle() < angle){
            clawSubsystem.setClawRotateSpeed(ClawConstants.CLAW_ROTATE_SPEED);
        }
        else{
            clawSubsystem.setClawRotateSpeed(-ClawConstants.CLAW_ROTATE_SPEED);;
        }

        //max do the PID thingy
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawRotateSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angle - clawSubsystem.getClawAngle()) < ClawConstants.ANGLE_DELTA;
    }
}
