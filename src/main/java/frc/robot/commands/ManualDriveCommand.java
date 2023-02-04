// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The command to drive the robot manually with joysticks.
 */
public class ManualDriveCommand extends CommandBase {
  private final DriveTrainSubsystem driveTrain;
  private final FlightJoystick joystick;
  private final LimeLightSubsystem limelight;

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveTrain The drivetrain subsystem.
   */
  public ManualDriveCommand(DriveTrainSubsystem driveTrain, FlightJoystick joystick, LimeLightSubsystem limelight) {
    this.driveTrain = driveTrain;
    this.joystick = joystick;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.joystick.getRawButtonWrapper(8)){
      Gyro.resetGyroAngle();
    }
    if (this.joystick.getRawButtonWrapper(1)){
      this.driveTrain.tankDrive(0, limelight.getAdjustment());
    }
    else{
      this.driveTrain.tankDriveAndMecanumDriveHaveAHorrificAmalgamationOfAChild(this.joystick.getHorizontalMovement(),-this.joystick.getLateralMovement());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
