package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

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

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
