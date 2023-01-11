package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawGripCommand extends CommandBase{

    private final ClawSubsystem clawSubsystem;
    private boolean action; //true -> open claw, false -> close claw

    public ClawGripCommand(ClawSubsystem clawSubsystem, boolean action){
        this.clawSubsystem = clawSubsystem;
        this.action = action;
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
