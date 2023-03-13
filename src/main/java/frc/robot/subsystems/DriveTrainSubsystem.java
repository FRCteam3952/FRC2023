// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.TrajectoryConstants;
import frc.robot.Constants.FieldConstants.AprilTagConstants;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.FlightJoystick;
import frc.robot.Constants.PortConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.util.MathUtil;
import frc.robot.util.NetworkTablesUtil;

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

    private final FlightJoystick joystick;

    private boolean swapDirection = false;

    private boolean blueTeam = NetworkTablesUtil.getIfOnBlueTeam();

    private final Encoder m_leftEncoder;
    private final Encoder m_rightEncoder;
    private static final int kEncoderResolution = 4096;

    private final DifferentialDrivePoseEstimator m_poseEstimator;

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

        this.m_leftEncoder = new Encoder(0, 1);
        this.m_rightEncoder = new Encoder(2, 3);

        this.m_leftEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.K_WHEEL_RADIUS / kEncoderResolution);
        this.m_rightEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.K_WHEEL_RADIUS / kEncoderResolution);

        this.m_leftEncoder.reset();
        this.m_rightEncoder.reset();

        this.m_poseEstimator = new DifferentialDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            RobotGyro.getRotation2d(),
            m_leftEncoder.getDistance(), 
            m_rightEncoder.getDistance(), 
            new Pose2d(), 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02,0.02,0.01), 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1,0.1,0.01)
        );

        this.joystick = joystick;

        this.tankDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
        tankDrive.setSafetyEnabled(false);


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

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.leftMotorGroup.setVoltage(leftVolts);
        this.rightMotorGroup.setVoltage(rightVolts);
        this.tankDrive.feed();
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
        m_poseEstimator.resetPosition(new Rotation2d(RobotGyro.getGyroAngleDegreesYaw()), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
    }

    public Pose2d getPoseMeters() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Pose2d getPoseInches() {
        Pose2d poseMeters = getPoseMeters();
        double conversionFactor = 39.3700787402;
        Pose2d poseInches = new Pose2d(poseMeters.getX() * conversionFactor, poseMeters.getY() * conversionFactor, poseMeters.getRotation());
        return poseInches;
    }

    public Command resetOdometryCommand(Pose2d pose) {
        return new InstantCommand(() -> resetOdometry(pose), this);
    }

    public void updateOdometry() {
        m_poseEstimator.update(RobotGyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance()); //update pose
    
        // Also apply vision measurements
        m_poseEstimator.addVisionMeasurement(
            NetworkTablesUtil.getJetsonPoseMeters(),
            Timer.getFPGATimestamp() - AprilTagConstants.LATENCY);
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

        /*if (RobotContainer.inTeleop) {
            if (Math.abs(this.joystick.getHorizontalMovement()) < 0.1 && Math.abs(this.joystick.getLateralMovement()) < 0.1) {
                var gyroRad = Math.toRadians(RobotGyro.getGyroAngleDegreesYaw());
                odometry.resetPosition(new Rotation2d(gyroRad), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), new Pose2d(NetworkTablesUtil.getJetsonPoseMeters(), new Rotation2d(gyroRad)));
            }
        }*/
        updateOdometry();

        Pose2d pose = getPoseMeters();
        double[] sendPose = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        NetworkTablesUtil.getEntry("robot", "drive_odometry").setDoubleArray(sendPose);

        // System.out.println("Gyro Yaw: " + RobotGyro.getGyroAngleDegreesYaw());
        // System.out.println("Gyro Roll: " + RobotGyro.getGyroAngleDegreesRoll());
        // System.out.println("Gyro Pitch: " + RobotGyro.getGyroAngleDegreesPitch());
        //System.out.println("Drive motor value: " + this.getWheelSpeeds());

        if (joystick.getRawButtonReleasedWrapper(ControllerConstants.RESET_GYRO_BUTTON_NUMBER)) {
            RobotGyro.resetGyroAngle();
        }

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