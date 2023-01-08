// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;


/**
 * self explanatory
 */

public class DriveTrain extends SubsystemBase {

  private static CANSparkMax frontLeftMotor;
  private static CANSparkMax frontRightMotor;
  private static CANSparkMax rearLeftMotor;
  private static CANSparkMax rearRightMotor;

  private static MotorControllerGroup leftMotorGroup;
  private static MotorControllerGroup rightMotorGroup;

  private final RelativeEncoder frontLeftEncoder;
  private final RelativeEncoder frontRightEncoder;
  private final RelativeEncoder rearLeftEncoder;
  private final RelativeEncoder rearRightEncoder;

  private final DifferentialDrive tankDrive;
  private final DifferentialDriveOdometry odometry;


  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {

    frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorPort, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorPort, MotorType.kBrushless);
    rearLeftMotor = new CANSparkMax(DriveConstants.rearLeftMotorPort, MotorType.kBrushless);
    rearRightMotor = new CANSparkMax(DriveConstants.rearRighttMotorPort, MotorType.kBrushless);
    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    rearLeftEncoder = rearLeftMotor.getEncoder();
    rearRightEncoder = rearRightMotor.getEncoder();

    frontLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderConversionFactor);
    frontRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderConversionFactor);
    rearLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderConversionFactor);
    rearRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderConversionFactor);

    leftMotorGroup = new MotorControllerGroup(frontLeftMotor, rearLeftMotor);
    rightMotorGroup = new MotorControllerGroup(frontRightMotor, rearRightMotor);

    frontRightMotor.setInverted(false);
    rearRightMotor.setInverted(false);
    frontLeftMotor.setInverted(true);
    rearLeftMotor.setInverted(true);

    odometry = new DifferentialDriveOdometry(null, 0, 0);

    tankDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    // m_dDrive.setSafetyEnabled(false);
    // resetEncoders();

  }

  public void tankDrive(double xSpeed, double zRotation) {
      //m_dDrive.driveCartesian(0, 0, 0, -Gyro.getGyroAngle());
      //5System.out.println("ySpeed: " + ySpeed + ", xSpeed: " + xSpeed + ", zRotation: " + zRotation);
    tankDrive.arcadeDrive(xSpeed, zRotation, false);
    /*
    frontLeft.set(0.2);
    frontRight.set(-0.2);
    rearLeft.set(-0.2);
    rearRight.set(0.2);
    */
    // if (RobotContainer.primaryJoystick.joystick.getRawButtonPressed(Constants.resetGyroButtonNumber)) {
    //   Gyro.resetGyroAngle();
    // }
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorGroup.setVoltage(leftVolts);
    rightMotorGroup.setVoltage(rightVolts);
    tankDrive.feed();
  }

  public double findZRotationSpeedFromAngle(double angle) {

    double angleDifference = angle - Gyro.getGyroAngle(); // gets angle difference

    if (Math.abs(angleDifference) >= 180) {
      /**
       * ensures that angleDifference is the smallest possible movement to the
       * destination
       */
      angleDifference = angleDifference + (angleDifference > 0 ? -360 : 360);
    }

    /**
     * positive angleDifference -> turn clockwise, negative angleDifference -> turn
     * counterclockwise
     * strength of turning power is proportional to size of angleDifference
     */
    double zRotation = angleDifference / 120;

    if (zRotation > 1) {
      zRotation = 1;
    } else if (zRotation < -1) {
      zRotation = -1;
    }

    return zRotation;
  }


  public double getPosition() {
    double[] encoderPositions = { frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(),
        rearLeftEncoder.getPosition(), rearRightEncoder.getPosition() };
    double sum = 0;

    for (double i : encoderPositions) {
      sum += i;
    }

    return sum / 4;
  }

  public double getFrontLeftEncoder() {
    // System.out.println(frontLeftEncoder.getPosition());
    return Math.abs(frontLeftEncoder.getPosition());
  }
  public double getFrontRightEncoder() {
    // System.out.println(frontLeftEncoder.getPosition());
    return Math.abs(frontRightEncoder.getPosition());
  }
  public double getRearLeftEncoder() {
    // System.out.println(frontLeftEncoder.getPosition());
    return Math.abs(rearLeftEncoder.getPosition());
  }
  public double getRearRightEncoder() {
    // System.out.println(frontLeftEncoder.getPosition());
    return Math.abs(rearRightEncoder.getPosition());
  }

  public void resetFrontLeftEncoder() {
    frontLeftEncoder.setPosition(0);
  }

  public void resetEncoders() {
      frontLeftEncoder.setPosition(0);
      frontRightEncoder.setPosition(0);
      rearLeftEncoder.setPosition(0);
      rearRightEncoder.setPosition(0);
  }

  public void setAllEncoders(double position) {
    frontLeftEncoder.setPosition(position);
    frontRightEncoder.setPosition(position);
    rearLeftEncoder.setPosition(position);
    rearRightEncoder.setPosition(position);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    
    odometry.update(new Rotation2d(Gyro.getGyroAngle()), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());

    //System.out.println("FL: " + getFrontLeftEncoder() + ", FR: " + getFrontRightEncoder() + ", RL: " + getRearLeftEncoder() + ", RR: " + getRearRightEncoder());
    //System.out.println("FL: " + frontLeft.get() + ", FR: " + frontRight.get() + ", RL: " + rearLeft.get() + ", RR: " + rearRight.get());

}

  @Override
  public void simulationPeriodic() {

  }

  public void stopMotors() {
    tankDrive.stopMotor();
  }

}