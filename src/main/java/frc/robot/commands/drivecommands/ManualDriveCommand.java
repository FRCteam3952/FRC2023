// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.FlightJoystick;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

/**
 * The command to drive the robot manually with joysticks.
 */
public class ManualDriveCommand extends CommandBase {
    private static final double MICRO_PP = 0.2; // Micro Pinpoint Positioning :tm:

    private final DriveTrainSubsystem driveTrain;
    private final FlightJoystick joystick;

    public ManualDriveCommand(DriveTrainSubsystem driveTrain, FlightJoystick joystick) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.joystick.getRawButtonWrapper(8)) {
            RobotGyro.resetGyroAngle();
        }
        this.driveTrain.tankDrive(-this.joystick.getVerticalMovement(), this.joystick.getRotation());
        // this.driveTrain.tankDriveAndMecanumDriveHaveAHorrificAmalgamationOfAChild(this.joystick.getHorizontalMovement(), this.joystick.getVerticalMovement());

        int pov = this.joystick.joystick.getHID().getPOV();
        if (pov == 0) {
            this.driveTrain.tankDrive(MICRO_PP, 0);
        } else if (pov == 180) {
            this.driveTrain.tankDrive(-MICRO_PP, 0);
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
