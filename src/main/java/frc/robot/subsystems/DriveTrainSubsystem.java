// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.TrajectoryConstants;
import frc.robot.controllers.FlightJoystick;
import frc.robot.Constants.PortConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.util.MathUtil;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.AprilTagUtil;

import java.util.List;

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
    private final RelativeEncoder[] encoders;

    private final DifferentialDrive tankDrive;
    private final DifferentialDriveOdometry odometry;

    private final FlightJoystick joystick;

    private boolean swapDirection = false;

    private boolean blueTeam = true; // We can toggle this with the slider on the drive joystick, TODO: implement later


    public DriveTrainSubsystem(FlightJoystick joystick) {
        this.frontLeftMotor = new CANSparkMax(PortConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
        this.frontRightMotor = new CANSparkMax(PortConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
        this.rearLeftMotor = new CANSparkMax(PortConstants.REAR_LEFT_MOTOR_PORT, MotorType.kBrushless);
        this.rearRightMotor = new CANSparkMax(PortConstants.REAR_RIGHT_MOTOR_PORT, MotorType.kBrushless);

        this.frontLeftEncoder = frontLeftMotor.getEncoder();
        this.frontRightEncoder = frontRightMotor.getEncoder();
        this.rearLeftEncoder = rearLeftMotor.getEncoder();
        this.rearRightEncoder = rearRightMotor.getEncoder();

        this.encoders = new RelativeEncoder[]{frontLeftEncoder, frontRightEncoder, rearLeftEncoder, rearRightEncoder};

        for (RelativeEncoder encoder : encoders) {
            encoder.setPositionConversionFactor(DriveConstants.ENCODER_CONVERSION_FACTOR);
        }

        this.leftMotorGroup = new MotorControllerGroup(frontLeftMotor, rearLeftMotor);
        this.rightMotorGroup = new MotorControllerGroup(frontRightMotor, rearRightMotor);

        this.frontRightMotor.setInverted(false);
        this.rearRightMotor.setInverted(false);
        this.frontLeftMotor.setInverted(true);
        this.rearLeftMotor.setInverted(true);

        this.odometry = new DifferentialDriveOdometry(new Rotation2d(RobotGyro.getGyroAngleDegreesYaw()), 0, 0);

        this.joystick = joystick;

        this.tankDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
        tankDrive.setSafetyEnabled(false);
        // m_dDrive.setSafetyEnabled(false);
        // resetEncoders();

    }

    /**
     * Wrapper around arcadeDrive
     *
     * @param xSpeed    The movement speed
     * @param zRotation The rotation speed
     */

    public void tankDrive(double xSpeed, double zRotation) {
        tankDrive.arcadeDrive(xSpeed, zRotation, false);
        // if (RobotContainer.primaryJoystick.joystick.getRawButtonPressed(Constants.resetGyroButtonNumber)) {
        //   Gyro.resetGyroAngle();
        // }
    }

    /**
     * Simulates a MecanumDrive on ArcadeDrive/TankDrive
     *
     * @param x The x movement speed
     * @param y The y movement speed
     */
    public void tankDriveAndMecanumDriveHaveAHorrificAmalgamationOfAChild(double x, double y) {
        if (x == 0 && y == 0) { // If no movement, make sure robot is stopped
            tankDrive.arcadeDrive(0, 0);
            return;
        }
        double speed = MathUtil.distance(0, x, 0, y); // Speed should take the distance to move into account
        double target = normalizeAngle(Math.toDegrees((Math.atan2(y, x))) - 90); // Normalize the target angle based on the slope from (0,0) to the point on the unit circle from joystick
        double current = swapDirection ? normalizeAngle(RobotGyro.getGyroAngleDegreesYaw() + 180) : normalizeAngle(RobotGyro.getGyroAngleDegreesYaw()); // Our current angle, normalized and accounting for if we're going "backwards"

        // The largest possible movement is 90 degrees because our robot is bi-directional (forwards or backwards does not matter on the tank drive)
        // Since the maximum distance from the x axis is 90 degrees, we check for the shortest angle between the two. If the smallest angle is greater than 90, we need to switch the side we're looking at.
        if (getShortestAngleApart(target, current) > 90) {
            swapDirection = !swapDirection; // Swap the direction
            current = normalizeAngle(current + 180); // And re-normalize our new angle
            double angleError = getAngleError(current, target); // Get the angle difference, which we now know to be the smallest.
            if (Math.abs(angleError) < DriveConstants.ANGLE_DELTA) { // If it's within the delta, we can stop to avoid jittering and indecisiveness.
                this.tankDrive(speed * (swapDirection ? -1 : 1), 0); // zRotation = 0, so no turning
            } else {
                double turningSpeed = angleError * DriveConstants.TURN_CONSTANT; // Scale the angleError to our turn constant
                // Make sure turningSpeed is at least 0.2 away from 0
                turningSpeed = Math.abs(turningSpeed) < 0.2 ? Math.copySign(0.2, turningSpeed) : turningSpeed;
                this.tankDrive(speed * (swapDirection ? -1 : 1), turningSpeed); // Drive
            }
        } else { // No swap necessary
            double angleError = getAngleError(current, target); // Same code as above, without the swap logic.
            if (Math.abs(angleError) < DriveConstants.ANGLE_DELTA) {
                this.tankDrive(speed * (swapDirection ? -1 : 1), 0);
            } else {
                double turningSpeed = -angleError * DriveConstants.TURN_CONSTANT;
                // Make sure turningSpeed is at least 0.2 away from 0
                turningSpeed = Math.abs(turningSpeed) < 0.2 ? Math.copySign(0.2, turningSpeed) : turningSpeed;
                this.tankDrive(speed * (swapDirection ? -1 : 1), turningSpeed);
            }
        }
    }


    public double normalizeAngle(double angle) {
        angle %= 360;
        angle = angle < 0 ? 360 + angle : angle;
        if (Math.abs(360 - angle) < 0.5) {
            angle = 0;
        }
        return angle;
    }


    public double getShortestAngleApart(double a1, double a2) {
        double difference = Math.abs(a1 - a2);
        return difference > 180 ? 360 - difference : difference;
    }


    public double getAngleError(double current, double target) {
        double angle = getShortestAngleApart(current, target);
        if (Math.abs(current + angle - target) < 0.25) {
            return angle;
        }
        if (Math.abs((target + angle) % 360 - current) < 0.25) {
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

        double angleDifference = angle - RobotGyro.getGyroAngleDegreesYaw(); // gets angle difference

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
        for (RelativeEncoder encoder : encoders) {
            encoder.setPosition(0);
        }
    }

    public void setAllEncoders(double position) {
        for (RelativeEncoder encoder : encoders) {
            encoder.setPosition(position);
        }
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(new Rotation2d(RobotGyro.getGyroAngleDegreesYaw()), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
    }

    public Pose2d getPoseMeters() {
        return odometry.getPoseMeters();
    }

    public Pose2d getPoseInches() {
        Pose2d poseMeters = odometry.getPoseMeters();
        double conversionFactor = 39.3700787402;
        Pose2d poseInches = new Pose2d(poseMeters.getX() * conversionFactor, poseMeters.getY() * conversionFactor, poseMeters.getRotation());
        return poseInches;
    }

    public Command resetOdometryCommand(Pose2d pose) {
        return new InstantCommand(() -> resetOdometry(pose), this);
    }

    // Generate command for following a trajectory
    public Command generateRamseteCommand(Pose2d startPoint, Pose2d endPoint, boolean reversed) {
        // A trajectory to follow. All units in meters.
        Trajectory trajectory = this.generateTrajectory(startPoint, List.of(), endPoint, reversed);

        return this.generateRamseteCommand(trajectory);
    }

    /**
     * A wrapper around {@link TrajectoryGenerator#generateTrajectory(Pose2d, List, Pose2d, TrajectoryConfig) that handles the config internally.
     * @param start The start Pose2d
     * @param waypoints A list of Translation2d waypoints to follow. Pass in {@link List#of()} if you don't want any waypoints.
     * @param end The end Pose2d
     * @return A trajectory. Use this to generate a follow command with {@link #generateRamseteCommand(Trajectory)}.
     */
    public Trajectory generateTrajectory(Pose2d start, List<Translation2d> waypoints, Pose2d end, boolean reversed) {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLTS_SECONDS_PER_METER, DriveConstants.KA_VOLTS_SECONDS_SQ_PER_METER), DriveConstants.DRIVE_KINEMATICS, 10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_METERS_PER_SECOND, TrajectoryConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.DRIVE_KINEMATICS)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                // Set reversed
                .setReversed(reversed);

        // A trajectory to follow. All units in meters.
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    }

    /**
     * Generates a command that will follow a trajectory. This command should be run immediately, and the given trajectory should begin at the current robot position.
     * @param trajectory the trajectory to follow
     * @return the command to follow the trajectory
     */
    
    public Command generateRamseteCommand(Trajectory trajectory) {
        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                this::getPoseMeters,
                new RamseteController(TrajectoryConstants.RAMSETE_B, TrajectoryConstants.RAMSETE_ZETA),
                new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLTS_SECONDS_PER_METER, DriveConstants.KA_VOLTS_SECONDS_SQ_PER_METER),
                DriveConstants.DRIVE_KINEMATICS,
                this::getWheelSpeeds,
                new PIDController(DriveConstants.P_DRIVE_VEL, 0, 0),
             new PIDController(DriveConstants.P_DRIVE_VEL, 0, 0),
                // RamseteCommand passes volts to the callback
                this::tankDriveVolts,
                this
        );

        // Reset odometry to the starting pose of the trajectory.
        // this.resetOdometry(trajectory.getInitialPose());

        // Reset odometry, run path following command, then stop at the end.
        return /*this.resetOdometryCommand(trajectory.getInitialPose()).andThen*/(ramseteCommand).andThen(() -> this.tankDriveVolts(0, 0), this);

        // ORIGINAL:
        // Run path following command, then stop at the end.
        // return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
    }

    @Override
    public void periodic() {
        odometry.update(new Rotation2d(Math.toRadians(RobotGyro.getGyroAngleDegreesYaw())), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());

        if (RobotContainer.inTeleop) {
            if (Math.abs(this.joystick.getHorizontalMovement()) < 0.1 && Math.abs(this.joystick.getLateralMovement()) < 0.1) {
                var gyroRad = Math.toRadians(RobotGyro.getGyroAngleDegreesYaw());
                odometry.resetPosition(new Rotation2d(gyroRad), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), new Pose2d(NetworkTablesUtil.getJetsonPoseMeters(), new Rotation2d(gyroRad)));
            }
        }

        Pose2d pose = odometry.getPoseMeters();
        double[] sendPose = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        NetworkTablesUtil.getEntry("robot", "drive_odometry").setDoubleArray(sendPose);

        // String currKey = NetworkTablesUtil.getKeyString();
        
        /*
        // Generates trajectories from the robot's current position to a specific April Tag and schedules them to be followed
        if (blueTeam) { // TODO: adjust tag id's to be correct
            switch (currKey) {
                case "q":
                    generateRamseteCommand(this.getPoseInches(), AprilTagUtil.poseOfTag2d(1), false).schedule();
                    break;
                case "w":
                    generateRamseteCommand(this.getPoseInches(), AprilTagUtil.poseOfTag2d(2), false).schedule();
                    break;
                case "e":
                    generateRamseteCommand(this.getPoseInches(), AprilTagUtil.poseOfTag2d(3), false).schedule();
                    break;
                default:
                    // System.out.println("No tag selected");
                    break;
            }
        } else {
            switch (currKey) {
                case "q":
                    generateRamseteCommand(this.getPoseInches(), AprilTagUtil.poseOfTag2d(6), false).schedule();
                    break;
                case "w":
                    generateRamseteCommand(this.getPoseInches(), AprilTagUtil.poseOfTag2d(7), false).schedule();
                    break;
                case "e":
                    generateRamseteCommand(this.getPoseInches(), AprilTagUtil.poseOfTag2d(8), false).schedule();
                    break;
                default:
                    // System.out.println("No tag selected");
                    break;
            }
        } */
        
        // var pose = odometry.getPoseMeters();

        // System.out.println("pose: " + pose.getX() + ", " + pose.getY() + ", " + pose.getRotation().getDegrees() + ", gyro: " + RobotGyro.getGyroAngleDegrees());

        // System.out.println("FL: " + getFrontLeftEncoder() + ", FR: " + getFrontRightEncoder() + ", RL: " + getRearLeftEncoder() + ", RR: " + getRearRightEncoder());
        // System.out.println("FL: " + frontLeft.get() + ", FR: " + frontRight.get() + ", RL: " + rearLeft.get() + ", RR: " + rearRight.get());
        

    }

    @Override
    public void simulationPeriodic() {

    }

    public void stopMotors() {
        tankDrive.stopMotor();
    }
}