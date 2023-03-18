// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivecommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * The command to drive the robot manually with joysticks.
 */
public class DriveToPosition extends CommandBase {

    private final DriveTrainSubsystem driveTrain;
    private double x;
    private double y;


    public DriveToPosition(DriveTrainSubsystem driveTrain, double x, double y) {
        this.driveTrain = driveTrain;
        this.x = x;
        this.y = y;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d curPose = driveTrain.getPoseMeters(); //sets the poses
        Pose2d finalPose = new Pose2d(this.x,this.y, new Rotation2d(0));

        driveTrain.generateRamseteCommand(curPose, finalPose, isFinished()); //generates trajectory and creates the ramsete command
    }

    // Called every time the scheduler runs while the command is scheduled.
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
