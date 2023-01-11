package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.InverseKinematicsUtil;
import frc.robot.Constants.ArmInverseKinematicsConstants;

public class MoveArmToAngleCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double x, y, z;
    private double[] targetAngles;
    private double[] currentAngles;

    public MoveArmToAngleCommand(ArmSubsystem armSubsystem, double x, double y, double z) {
        this.armSubsystem = armSubsystem;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public void initialize() {
        this.targetAngles = InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z);
    }

    @Override
    public void execute() {
        this.currentAngles = armSubsystem.getAnglefromEncoder();

        // if the angle for that particular pivot is within delta, then stop it
        // otherwise, move it in the proper direction
        if(Math.abs(this.currentAngles[0] - this.targetAngles[0]) < ArmInverseKinematicsConstants.ANGLE_DELTA) {
            this.armSubsystem.setPivot1Speed(0);
        } else {
            this.armSubsystem.setPivot1Speed(ArmInverseKinematicsConstants.PIVOT_SPEED * (this.currentAngles[0] > this.targetAngles[0] ? 1 : -1));
        }

        if(Math.abs(this.currentAngles[1] - this.targetAngles[1]) < ArmInverseKinematicsConstants.ANGLE_DELTA) {
            this.armSubsystem.setPivot2Speed(0);
        } else {
            this.armSubsystem.setPivot2Speed(ArmInverseKinematicsConstants.PIVOT_SPEED * (this.currentAngles[1] > this.targetAngles[1] ? 1 : -1));
        }
        
        if(Math.abs(this.currentAngles[2] - this.targetAngles[2]) < ArmInverseKinematicsConstants.ANGLE_DELTA) {
            this.armSubsystem.setTurretSpeed(0);
        } else {
            this.armSubsystem.setTurretSpeed(ArmInverseKinematicsConstants.TURRET_SPEED * (this.currentAngles[2] > this.targetAngles[2] ? 1 : -1));
        }

        // max do pid later
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.stopAllMotors();
    }

    @Override
    public boolean isFinished() {
        for(int i = 0; i < this.targetAngles.length; i++) {
            if(Math.abs(this.targetAngles[i] - this.currentAngles[i]) > ArmInverseKinematicsConstants.ANGLE_DELTA) {
                return false;
            }
        }
        return true;
    }
}