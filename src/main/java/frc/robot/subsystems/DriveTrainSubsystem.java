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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.TrajectoryConstants;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.controllers.FlightJoystick;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.Constants.FieldConstants.GamePiecePlacementLocationConstants;


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

    //private boolean swapDirection = false; commented because I don't like yellow file

    //private boolean blueTeam = NetworkTablesUtil.getIfOnBlueTeam();

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
            encoder.setVelocityConversionFactor(DriveConstants.ENCODER_CONVERSION_FACTOR);
        }

        resetEncoders();

        this.leftMotorGroup = new MotorControllerGroup(frontLeftMotor, rearLeftMotor);
        this.rightMotorGroup = new MotorControllerGroup(frontRightMotor, rearRightMotor);

        this.frontRightMotor.setInverted(true);
        this.rearRightMotor.setInverted(true);
        this.frontLeftMotor.setInverted(false);
        this.rearLeftMotor.setInverted(false);

        this.m_poseEstimator = new DifferentialDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS,
                RobotGyro.getRotation2d(),
                frontLeftEncoder.getPosition(),
                frontRightEncoder.getPosition(),
                new Pose2d(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
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
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        //System.out.println("L VOLTS: " + leftVolts + ", R VOLTS; " + rightVolts);
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
        return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity() / 60d, frontRightEncoder.getVelocity() / 60d);
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
        m_poseEstimator.update(RobotGyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition()); //update pose

        // Also apply vision measurements
        // m_poseEstimator.addVisionMeasurement(
        //     NetworkTablesUtil.getJetsonPoseMeters(),
        //    Timer.getFPGATimestamp() - AprilTagConstants.LATENCY);
    }

    /**
     * The camera is slightly offset from the center of the robot. This needs to be accounted for in the real robot pose.
     */
    public Pose2d getCenteredJetsonPose() {
        var pose = NetworkTablesUtil.getJetsonPoseMeters();
        double xShift = Math.cos(Math.toRadians(RobotGyro.getGyroAngleDegreesYaw())) * RobotConstants.CAMERA_SIDE_OFFSET_FROM_CENTER_M;
        double zShift = Math.sin(Math.toRadians(RobotGyro.getGyroAngleDegreesYaw())) * RobotConstants.CAMERA_SIDE_OFFSET_FROM_CENTER_M;

        return new Pose2d(pose.getX() + xShift, pose.getY() + zShift, pose.getRotation());
    }

    // Generate command for following a trajectory
    public Command generateRamseteCommand(Pose2d startPoint, Pose2d endPoint, boolean reversed) {
        // A trajectory to follow. All units in meters.
        Trajectory trajectory = this.generateTrajectory(startPoint, List.of(), endPoint, reversed);
        return this.generateRamseteCommand(trajectory);
    }

    /**
     * A wrapper around {@link TrajectoryGenerator#generateTrajectory(Pose2d, List, Pose2d, TrajectoryConfig) that handles the config internally.
     *
     * @param start     The start Pose2d
     * @param waypoints A list of Translation2d waypoints to follow. Pass in {@link List#of()} if you don't want any waypoints.
     * @param end       The end Pose2d
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
     *
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
        return this.resetOdometryCommand(trajectory.getInitialPose()).andThen(ramseteCommand).andThen(() -> this.tankDriveVolts(0, 0), this);

        // ORIGINAL:
        // Run path following command, then stop at the end.
        // return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
    }
    
    public static final String PoseUnit = "IN";
    //Use "IN" or "M" for inches or meters

    public Pose2d getStartPose() {
        if(PoseUnit == "IN") {
            return getPoseInches();
        }
        else if(PoseUnit == "M") {
            return getPoseMeters();
        }
        return new Pose2d();
    }

    public Pose2d getEndPose() {
        int idx = (int) NetworkTablesUtil.getEntry("robogui","selectedPlacementPosition").getInteger(0);
        
        int x_col = idx % 9;
        int z_row = x_col % 3;
        
        if(z_row==0) { //TODO: What to do with ground nodes
            return new Pose2d();
        }

        Pose3d tmp;
        switch(x_col) {
            // POLES:
            case 0:
                tmp = GamePiecePlacementLocationConstants.POLE_POSITIONS[z_row][0];
                break;
            case 2:
                tmp = GamePiecePlacementLocationConstants.POLE_POSITIONS[z_row][1];
                break;
            case 3:
                tmp = GamePiecePlacementLocationConstants.POLE_POSITIONS[z_row][2];
                break;
            case 5:
                tmp = GamePiecePlacementLocationConstants.POLE_POSITIONS[z_row][3];
                break;
            case 6:
                tmp = GamePiecePlacementLocationConstants.POLE_POSITIONS[z_row][4];
                break;
            case 8:
                tmp = GamePiecePlacementLocationConstants.POLE_POSITIONS[z_row][5];
                break;
            
            // PLATFORMS:
            case 1:
                tmp = GamePiecePlacementLocationConstants.PLATFORM_POSITIONS[z_row][0];
                break;
            case 4:
                tmp = GamePiecePlacementLocationConstants.PLATFORM_POSITIONS[z_row][1];
            case 7:
                tmp = GamePiecePlacementLocationConstants.PLATFORM_POSITIONS[z_row][2];
            default:
                tmp = new Pose3d();
                System.out.println("something broke...NetworkTables did not provide the correct platform/pole index.");
                
        }
        return new Pose2d(
                    tmp.getX(),
                    tmp.getY(),
                    tmp.getRotation().toRotation2d()
                );
    } 


    @Override
    public void periodic() {
        //get starting pose         
        updateOdometry();

        Pose2d pose = getPoseMeters();
        double[] sendPose = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        NetworkTablesUtil.getEntry("robot", "drive_odometry").setDoubleArray(sendPose);

        // System.out.println("Gyro Yaw: " + RobotGyro.getGyroAngleDegreesYaw());
        // System.out.println("Gyro Roll: " + RobotGyro.getGyroAngleDegreesRoll());
        // System.out.println("Gyro Pitch: " + RobotGyro.getGyroAngleDegreesPitch());
        //System.out.println("Drive motor value: " + this.getWheelSpeeds());

        if (Joystick.getRawButtonReleasedWrapper(ControllerConstants.RESET_GYRO_BUTTON_NUMBER)) {
            RobotGyro.resetGyroAngle();
        }

        NetworkTablesUtil.getConnections();

        //System.out.println("FL: " + frontLeftEncoder.getPosition() + ", FR: " + frontRightEncoder.getPosition() + ", RL: " + rearLeftEncoder.getPosition() + ", RR: " + rearRightEncoder.getPosition());

        // String currKey = NetworkTablesUtil.getKeyString();

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