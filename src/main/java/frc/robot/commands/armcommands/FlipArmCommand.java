package frc.robot.commands.armcommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

public class FlipArmCommand extends CommandBase {
    private final ArmSubsystem arm;
    private boolean flipped = true;

    private boolean canBeginFlip = false;

    private boolean end = false;

    public FlipArmCommand(ArmSubsystem arm, boolean flipped) {
        this.arm = arm;
        this.flipped = flipped;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if(!flipped) {
            arm.moveVector(0, 10, 0); //move arm up 10 inches
        } else {
            arm.setArm1SpeedMultiplier(ArmConstants.COMPLEMENTING_FLIP_SPEED); // set speed multipliers 
            arm.setArm2SpeedMultiplier(ArmConstants.SPEED_DEC_ON_FLIP);
            arm.setTargetAngle1(ArmConstants.FLIPPING_TARGET_ANGLES[0]); // move arm to pre-defined flip coordinates (optimized for cone grabbing)
            arm.setTargetAngle2(ArmConstants.FLIPPING_TARGET_ANGLES[1]);
        }
    }

    @Override
    public void execute() {
        System.out.println("Arm is moving to position");
        if(flipped && arm.isAtCoords()){
            System.out.println("Arm is now FLIPPED!");
            end = true;
        }
        if(!canBeginFlip && !flipped && arm.isAtCoords()){ // waits for arm to move up 10 before flipping back upright
            System.out.println("Arm has moved up 10 inches");
            canBeginFlip = true;
            arm.setArm1SpeedMultiplier(ArmConstants.COMPLEMENTING_FLIP_SPEED); // set speed multipliers
            arm.setArm2SpeedMultiplier(ArmConstants.SPEED_DEC_ON_UNFLIP);
            arm.setTargetAngle1(ArmConstants.ARM_1_INITIAL_ANGLE);
            arm.setTargetAngle2(ArmConstants.ARM_2_INITIAL_ANGLE);
        }
        if(canBeginFlip && arm.isAtCoords()){
            System.out.println("Arm is now at starting config");
            end = true;
        }
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //resets multipliers to default
        arm.setArm1SpeedMultiplier(1d);
        arm.setArm2SpeedMultiplier(1d);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return end;
    }
}
