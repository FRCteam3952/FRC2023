package frc.robot.commands.armcommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

public class FlipArmCommand extends CommandBase {
    private final ArmSubsystem arm;
    private boolean flipped = true;
    private final PIDController pidController1, pidController2;

    private double[] initialCoordinates;

    private boolean canBeginFlip = false;

    private boolean end = false;

    public FlipArmCommand(ArmSubsystem arm, boolean flipped) {
        this.arm = arm;
        this.flipped = flipped;

        this.pidController1 = arm.getPID1();
        this.pidController2 = arm.getPID2();

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        initialCoordinates = arm.getCurrentCoordinates();

        if(!flipped) {
            arm.moveVector(0, 10, 0);
            arm.setTargetAngle1(ArmConstants.ARM_1_INITIAL_ANGLE);
            arm.setTargetAngle2(ArmConstants.ARM_2_INITIAL_ANGLE);
        } else {
            arm.setPIDControlState(false);
            arm.setTargetAngle1(ArmConstants.FLIPPING_TARGET_ANGLES[0]);
            arm.setTargetAngle2(ArmConstants.FLIPPING_TARGET_ANGLES[1]);
        }
    }

    @Override
    public void execute() {
        System.out.println("FLIPPING ARM COMMAND, FLIPPED=" + flipped);
        if(flipped) {
            if(Math.abs(arm.getCurrentAnglesDeg()[0] - ArmConstants.FLIPPING_TARGET_ANGLES[0]) > ArmConstants.ANGLE_DELTA || Math.abs(arm.getCurrentAnglesDeg()[1] - ArmConstants.FLIPPING_TARGET_ANGLES[1]) > ArmConstants.ANGLE_DELTA) {
                System.out.println("IN FLIP LOOP");
                double[] angles = arm.getCurrentAnglesDeg(); // gets the current angles read from motor encoders

                if (Double.isNaN(angles[0]) || Double.isNaN(angles[1]) || Double.isNaN(angles[2]) || Double.isNaN(ArmConstants.FLIPPING_TARGET_ANGLES[0]) || Double.isNaN(ArmConstants.FLIPPING_TARGET_ANGLES[1])) {
                    System.out.println("An angle is NaN, so skip");
                    return;
                }

                // gets PID control calculations
                double p1Speed = pidController1.calculate(angles[0], ArmConstants.FLIPPING_TARGET_ANGLES[0]);
                double p2Speed = pidController2.calculate(angles[1], ArmConstants.FLIPPING_TARGET_ANGLES[1]);

                // We've just flipped, one arm segment needs to go slower than the other segment to avoid ground collision
                // If we're going to flipped=true, then arm2 needs to go slower.
                // If we're going to flipped=false, then arm1 needs to go slower
                p2Speed *= ArmConstants.SPEED_DEC_ON_FLIP;
                p1Speed *= ArmConstants.COMPLEMENTING_FLIP_SPEED;

                // if power is NaN, don't run it :D
                if (Double.isNaN(p1Speed) || Double.isNaN(p2Speed)) {
                    System.out.println("PID is NaN, so skip");
                    return;
                }

                p1Speed = Math.min(ArmConstants.MAX_OUTPUT, Math.max(p1Speed, ArmConstants.MIN_OUTPUT));
                p2Speed = Math.min(ArmConstants.MAX_OUTPUT, Math.max(p2Speed, ArmConstants.MIN_OUTPUT));
                System.out.println("SPEEDS: " + p1Speed + " " + p2Speed);

                arm.setPivot1Speed(p1Speed);
                arm.setPivot2Speed(p2Speed);
            } else {
                end = true;
            }
        } else {
            if(!canBeginFlip && Math.abs(initialCoordinates[1] - arm.getCurrentCoordinates()[1]) > 5) {
                System.out.println("WAITING FOR CAN BEGIN FLIP");
                arm.setPIDControlState(false);
                canBeginFlip = true;
            }
            if(canBeginFlip) {
                if(Math.abs(arm.getCurrentAnglesDeg()[0] - ArmConstants.ARM_1_INITIAL_ANGLE) > ArmConstants.ANGLE_DELTA || Math.abs(arm.getCurrentAnglesDeg()[1] - ArmConstants.ARM_2_INITIAL_ANGLE) > ArmConstants.ANGLE_DELTA ) {
                    System.out.println("IN UNFLIP LOOP");
                    double[] angles = arm.getCurrentAnglesDeg(); // gets the current angles read from motor encoders
    
                    if (Double.isNaN(angles[0]) || Double.isNaN(angles[1]) || Double.isNaN(angles[2])) {
                        System.out.println("An angle is NaN, so skip");
                        return;
                    }
    
                    // gets PID control calculations
                    double p1Speed = pidController1.calculate(angles[0], ArmConstants.ARM_1_INITIAL_ANGLE);
                    double p2Speed = pidController2.calculate(angles[1], ArmConstants.ARM_2_INITIAL_ANGLE);
    
                    // We've just flipped, one arm segment needs to go slower than the other segment to avoid ground collision
                    // If we're going to flipped=true, then arm2 needs to go slower.
                    // If we're going to flipped=false, then arm1 needs to go slower
                    p1Speed *= ArmConstants.SPEED_DEC_ON_UNFLIP;
                    p2Speed *= ArmConstants.COMPLEMENTING_FLIP_SPEED;
    
                    // if power is NaN, don't run it :D
                    if (Double.isNaN(p1Speed) || Double.isNaN(p2Speed)) {
                        System.out.println("PID is NaN, so skip");
                        return;
                    }
    
                    p1Speed = Math.min(ArmConstants.MAX_OUTPUT, Math.max(p1Speed, ArmConstants.MIN_OUTPUT));
                    p2Speed = Math.min(ArmConstants.MAX_OUTPUT, Math.max(p2Speed, ArmConstants.MIN_OUTPUT));
                    System.out.println("SPEEDS: " + p1Speed + " " + p2Speed);
    
                    arm.setPivot1Speed(p1Speed);
                    arm.setPivot2Speed(p2Speed);
                } else {
                    end = true;
                }
            }
        }
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        arm.setPIDControlState(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return end;
    }
}
