// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The command to drive the robot manually with joysticks.
 */
public class ManualDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveTrain;
  private final FlightJoystick joystick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveTrain The drivetrain subsystem.
   */
  public ManualDrive(DriveTrain driveTrain, FlightJoystick joystick) {
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
    this.driveTrain.drive(this.joystick.getLateralMovement(), this.joystick.getRotation());
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
