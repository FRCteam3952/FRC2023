// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.util.InverseKinematicsUtil;

public class DriveTrainSubsystem extends SubsystemBase {

  private final CANSparkMax frontLeftMotor;
  private final CANSparkMax frontRightMotor;
  private final CANSparkMax rearLeftMotor;
  private final CANSparkMax rearRightMotor;

  private final MotorControllerGroup leftMotorGroup;
  private final MotorControllerGroup rightMotorGroup;

  private final RelativeEncoder frontLeftEncoder;
  private final RelativeEncoder frontRightEncoder;
  private final RelativeEncoder rearLeftEncoder;
  private final RelativeEncoder rearRightEncoder;

  private final DifferentialDrive tankDrive;
  private final DifferentialDriveOdometry odometry;

  private boolean swapDirection = false;

  public DriveTrainSubsystem() {
    this.frontLeftMotor = new CANSparkMax(PortConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
    this.frontRightMotor = new CANSparkMax(PortConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    this.rearLeftMotor = new CANSparkMax(PortConstants.REAR_LEFT_MOTOR_PORT, MotorType.kBrushless);
    this.rearRightMotor = new CANSparkMax(PortConstants.REAR_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    this.frontLeftEncoder = frontLeftMotor.getEncoder();
    this.frontRightEncoder = frontRightMotor.getEncoder();
    this.rearLeftEncoder = rearLeftMotor.getEncoder();
    this.rearRightEncoder = rearRightMotor.getEncoder();

    this.frontLeftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_CONVERSION_FACTOR);
    this.frontRightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_CONVERSION_FACTOR);
    this.rearLeftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_CONVERSION_FACTOR);
    this.rearRightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_CONVERSION_FACTOR);

    this.leftMotorGroup = new MotorControllerGroup(frontLeftMotor, rearLeftMotor);
    this.rightMotorGroup = new MotorControllerGroup(frontRightMotor, rearRightMotor);

    this.frontRightMotor.setInverted(false);
    this.rearRightMotor.setInverted(false);
    this.frontLeftMotor.setInverted(true);
    this.rearLeftMotor.setInverted(true);

    this.odometry = new DifferentialDriveOdometry(new Rotation2d(Gyro.getGyroAngle()), 0, 0);

    this.tankDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    // m_dDrive.setSafetyEnabled(false);
    // resetEncoders();

  }

  /**
   * Runs whatever the real drive method is
   * @param xSpeed
   * @param zRotation
   */

  public void tankDrive(double xSpeed, double zRotation) {
    tankDrive.arcadeDrive(xSpeed, zRotation, false);
    // if (RobotContainer.primaryJoystick.joystick.getRawButtonPressed(Constants.resetGyroButtonNumber)) {
    //   Gyro.resetGyroAngle();
    // }
  }

  public void tankDriveAndMecanumDriveHaveAHorrificAmalgamationOfAChild(double x, double y) {
    if(x == 0 && y == 0){
      tankDrive.arcadeDrive(0, 0);
      return;
    }
    double speed = InverseKinematicsUtil.distance(0, x, 0, y);
    double target = normalizeAngle((Math.atan2(y,x) * 180 / Math.PI) - 90);
    double current = swapDirection?normalizeAngle(Gyro.getGyroAngle()+180):normalizeAngle(Gyro.getGyroAngle());

    double turningSpeed = DriveConstants.TURN_CONSTANT / speed;

    if(shortestAngleApart(target, current) > 90){
      swapDirection = !swapDirection; 
      current = normalizeAngle(current+180);
      double angleError = getAngleError(current, target);
      if (Math.abs(angleError) < DriveConstants.ANGLE_DELTA){
        tankDrive.arcadeDrive(speed * (swapDirection?-1:1), 0, false);
      }
      else{
        tankDrive.arcadeDrive(speed * (swapDirection?-1:1), angleError > 0?turningSpeed:-turningSpeed, false);
      }
      System.out.println(current+ " " + angleError + " " + swapDirection);
    }
    else{
      double angleError = getAngleError(current, target);
      if (Math.abs(angleError) < DriveConstants.ANGLE_DELTA){
        tankDrive.arcadeDrive(speed * (swapDirection?-1:1), 0, false);
      }
      else{
        tankDrive.arcadeDrive(speed * (swapDirection?-1:1), angleError > 0?-turningSpeed:turningSpeed, false);
      }
      System.out.println(current+ " " + angleError + " " + swapDirection);

    }

  }
  public double normalizeAngle(double angle){
    angle %= 360;
    angle = angle < 0 ? 360+angle : angle;
    if (Math.abs(360-angle) < 0.5){
      angle = 0;
    }
    return angle;
  }
  public double shortestAngleApart(double a1, double a2){
    double difference = Math.abs(a1-a2);
    return difference>180?360-difference:difference;
  }
  public double getAngleError(double current, double target){
    double angle = shortestAngleApart(current, target);
    if(Math.abs(current + angle - target) < 0.25) {
      return angle;
    }
    if(Math.abs((target + angle) % 360 - current) < 0.25) {
      return -angle;
    }
    return angle;
  }
  

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    this.leftMotorGroup.setVoltage(leftVolts);
    this.rightMotorGroup.setVoltage(rightVolts);
    this.tankDrive.feed();
  }

  public double findZRotationSpeedFromAngle(double angle) {

    double angleDifference = angle - Gyro.getGyroAngle(); // gets angle difference

    if (Math.abs(angleDifference) >= 180) {
      /*
       * ensures that angleDifference is the smallest possible movement to the
       * destination
       */
      angleDifference = angleDifference + (angleDifference > 0 ? -360 : 360);
    }

    /*
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

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
      new Rotation2d(Gyro.getGyroAngle()), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // Generate command for following a trajectory
  public Command generateRamseteCommand(Pose2d startPoint, Pose2d endPoint) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    DriveConstants.KS_VOLTS,
                    DriveConstants.KV_VOLTS_SECONDS_PER_METER,
                    DriveConstants.KA_VOLTS_SECONDS_SQ_PER_METER),
            DriveConstants.DRIVE_KINEMATICS,
            10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
            TrajectoryConstants.MAX_SPEED_METERS_PER_SECOND,
            TrajectoryConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // A trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            startPoint,
            List.of(),
            endPoint,
            config);


    RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            this::getPose,
            new RamseteController(TrajectoryConstants.RAMSETE_B, TrajectoryConstants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                    DriveConstants.KS_VOLTS,
                    DriveConstants.KV_VOLTS_SECONDS_PER_METER,
                    DriveConstants.KA_VOLTS_SECONDS_SQ_PER_METER),
            DriveConstants.DRIVE_KINEMATICS,
            this::getWheelSpeeds,
            new PIDController(DriveConstants.P_DRIVE_VEL, 0, 0),
            new PIDController(DriveConstants.P_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            this::tankDriveVolts,
            this);

    // Reset odometry to the starting pose of the trajectory.
    this.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));

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