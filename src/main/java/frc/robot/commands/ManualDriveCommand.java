// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrappers.NetworkTables;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

/**
 * The command to drive the robot manually with joysticks.
 */
public class ManualDriveCommand extends CommandBase {
  private final DriveTrainSubsystem driveTrain;
  private final FlightJoystick joystick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveTrain The drivetrain subsystem.
   */
  public ManualDriveCommand(DriveTrainSubsystem driveTrain, FlightJoystick joystick, LimeLightSubsystem limelight) {
    this.driveTrain = driveTrain;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
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
    this.driveTrain.tankDriveAndMecanumDriveHaveAHorrificAmalgamationOfAChild(this.joystick.getHorizontalMovement(),-this.joystick.getLateralMovement());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Returns Jetson coordinates.
  public static float getJetsonCoordinateX() {
    float tx = (NetworkTables.getJetsonCoordinateXError() - 160) * kp; // Not sure if the kp is supposed to be there
    
    // if tx is too big, return the max of 1 or -1 (We might need to change this and the others too because I don't know what values are needed)
    if (Math.abs(tx) > 1) {
        // return 1 if tx is greater than 1, -1 if tx is less than -1
        return Math.copySign(1, tx);
    }
    return tx;
  }

  public static float getJetsonCoordinateY() {
    float ty = (NetworkTables.getJetsonCoordinateYError() - 120) * kp;

    // if ty is too big, return the max of 1 or -1
    if (Math.abs(ty) > 1) {
        // return 1 if ty is greater than 1, -1 if ty is less than -1
        return Math.copySign(1, ty);
    }
    return ty;
  }

  public static float getJetsonCoordinateZ() {
    float tz = (NetworkTables.getJetsonCoordinateZError() - 120) * kp;

    // if ty is too big, return the max of 1 or -1
    if (Math.abs(tz) > 1) {
        // return 1 if ty is greater than 1, -1 if ty is less than -1
        return Math.copySign(1, tz);
    }
    return tz;
  }
}
