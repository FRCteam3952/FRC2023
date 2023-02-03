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
  private final CANSparkMax[] motors;
  private final RelativeEncoder[] encoders;
  private final MotorControllerGroup[] motorGroups;

  private final DifferentialDrive tankDrive;
  private final DifferentialDriveOdometry odometry;

  private boolean swapDirection = false;

  public DriveTrainSubsystem() {
    // instantiate all motors based on their port constants into `this.motors`
    // to get a specific motor, use DriveConstants, e.g. motors[DriveConstants.FRONT_LEFT]
    motors = new CANSparkMax[4];
    encoders = new RelativeEncoder[4];
    for (int i = 0; i < this.motors.length; i++) {
      // create the motor and set correct inversion, more in PortConstants
      motors[i] = new CANSparkMax(PortConstants.ALL_MOTOR_PORTS[i], MotorType.kBrushless);
      motors[i].setInverted(DriveConstants.INVERSIONS[i]);
      // instantiate encoder for motor
      encoders[i] = this.motors[i].getEncoder();
      encoders[i].setPositionConversionFactor(DriveConstants.ENCODER_CONVERSION_FACTOR);
    }

    // motor groups, 0 is left and 1 is right
    motorGroups = new MotorControllerGroup[]{
      new MotorControllerGroup(motors[DriveConstants.FRONT_LEFT],  motors[DriveConstants.REAR_LEFT]),
      new MotorControllerGroup(motors[DriveConstants.FRONT_RIGHT], motors[DriveConstants.REAR_RIGHT])
    };

    this.odometry = new DifferentialDriveOdometry(new Rotation2d(Gyro.getGyroAngle()), 0, 0);
    this.tankDrive = new DifferentialDrive(motorGroups[0], motorGroups[1]);
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
    if(x == 0 && y == 0) { // If no movement, make sure robot is stopped
      tankDrive.arcadeDrive(0, 0);
      return;
    }
    double speed = InverseKinematicsUtil.distance(0, x, 0, y); // Speed should take the distance to move into account
    double target = normalizeAngle((Math.atan2(y,x) * 180 / Math.PI) - 90); // Normalize the target angle based on the slope from (0,0) to the point on the unit circle from joystick
    double current = swapDirection?normalizeAngle(Gyro.getGyroAngle()+180):normalizeAngle(Gyro.getGyroAngle()); // Our current angle, normalized and accounting for if we're going "backwards"

    // The largest possible movement is 90 degrees because our robot is bi-directional (forwards or backwards does not matter on the tank drive)
    // Since the maximum distance from the x axis is 90 degrees, we check for the shortest angle between the two. If the smallest angle is greater than 90, we need to switch the side we're looking at.
    if(getShortestAngleApart(target, current) > 90) {
      swapDirection = !swapDirection; // Swap the direction
      current = normalizeAngle(current+180); // And re-normalize our new angle
      double angleError = getAngleError(current, target); // Get the angle difference, which we now know to be the smallest.
      if (Math.abs(angleError) < DriveConstants.ANGLE_DELTA) { // If it's within the delta, we can stop to avoid jittering and indecisiveness.
        this.tankDrive(speed * (swapDirection?-1:1), 0); // zRotation = 0, so no turning
      } else {
        double turningSpeed = angleError * DriveConstants.TURN_CONSTANT; // Scale the angleError to our turn constant
        // Make sure turningSpeed is at least 0.2 away from 0
        turningSpeed = Math.abs(turningSpeed) < 0.2 ? Math.copySign(0.2, turningSpeed) : turningSpeed;
        this.tankDrive(speed * (swapDirection?-1:1), turningSpeed); // Drive
      }
    } else { // No swap necessary
      double angleError = getAngleError(current, target); // Same code as above, without the swap logic.
      if (Math.abs(angleError) < DriveConstants.ANGLE_DELTA) {
        this.tankDrive(speed * (swapDirection?-1:1), 0);
      } else {
        double turningSpeed = -angleError * DriveConstants.TURN_CONSTANT;
        // Make sure turningSpeed is at least 0.2 away from 0
        turningSpeed = Math.abs(turningSpeed) < 0.2 ? Math.copySign(0.2, turningSpeed) : turningSpeed;
        this.tankDrive(speed * (swapDirection?-1:1), turningSpeed);
      }
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


  public double getShortestAngleApart(double a1, double a2){
    double difference = Math.abs(a1-a2);
    return difference>180?360-difference:difference;
  }


  public double getAngleError(double current, double target){
    double angle = getShortestAngleApart(current, target);
    if(Math.abs(current + angle - target) < 0.25) {
      return angle;
    }
    if(Math.abs((target + angle) % 360 - current) < 0.25) {
      return -angle;
    }
    return angle;
  }
  

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    for(int i=0;i<2;i++) this.motorGroups[i].setVoltage(new double[]{ leftVolts, rightVolts }[i]);
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
    for (RelativeEncoder encoder : encoders) encoder.setPosition(0);
  }

  public void setAllEncoders(double position) {
    for (RelativeEncoder encoder : encoders) encoder.setPosition(position);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
      new Rotation2d(Gyro.getGyroAngle()), 
      encoders[DriveConstants.FRONT_LEFT].getPosition(), 
      encoders[DriveConstants.FRONT_RIGHT].getPosition(), 
      pose
    );
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      encoders[DriveConstants.FRONT_LEFT].getVelocity(), 
      encoders[DriveConstants.FRONT_RIGHT].getVelocity()
    );
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

  public Command followTrajectoryCommand(Trajectory trajectory) {
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
    
    odometry.update(
      new Rotation2d(Gyro.getGyroAngle()), 
      encoders[DriveConstants.FRONT_LEFT].getPosition(), 
      encoders[DriveConstants.FRONT_RIGHT].getPosition()
    );

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