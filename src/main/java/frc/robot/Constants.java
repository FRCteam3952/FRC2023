// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ArmInverseKinematicsConstants {

    // All constants are in inches
    public static final double ORIGIN_HEIGHT = 35.36; 
    public static final double LIMB1_LENGTH = 25.8;
    public static final double LIMB2_LENGTH = 20;

    public static final double ANGLE_DELTA = 2;

    public static final double PIVOT_SPEED = 0.1;
    public static final double TURRET_SPEED = 0.1;

    public static final double STARTING_X = 0;
    public static final double STARTING_Y = 0;
    public static final double STARTING_Z = 0;
  }

  public static class ClawConstants {
    public static final double CLAW_ROTATE_SPEED = 0.1;
    public static final double ANGLE_DELTA = 2;
  }

  public static class OperatorConstants {
    public static final int RIGHT_JOYSTICK_PORT = 0;
    public static final int LEFT_JOYSTICK_PORT = 1;
  }

  public static class ControllerConstants {
    // Joystick 0
    public static final int RUN_GUI_TRAJECTORY_BUTTON_NUMBER = 4; // idk i just picked a random number
    // Joystick 1
    public static final int AIM_ASSIST_BUTTON_NUMBER = 1; //change to whatever it is
    public static final int MOVE_ARM_UP_BUTTON_NUMBER = 2; //change to whatever it is
    public static final int MOVE_ARM_DOWN_BUTTON_NUMBER = 3; //change to whatever it is
  }

  public static class PortConstants {
    public static final int FRONT_RIGHT_MOTOR_PORT = 1;
    public static final int REAR_RIGHT_MOTOR_PORT = 2;
    public static final int REAR_LEFT_MOTOR_PORT = 3;
    public static final int FRONT_LEFT_MOTOR_PORT = 4;

    public static final int PIVOT1_PORT = 5;
    public static final int PIVOT2_PORT = 6;
    public static final int TURRET_PORT = 7;

    public static final int CLAW_GRIP_PORT = 8;
    public static final int CLAW_ROTATE_PORT = 9;
  }

  public static class DriveConstants {// All constants below are examples; must correct to robot's specification when it is finished being built
    public static final double KS_VOLTS = 0.22;
    public static final double KV_VOLTS_SECONDS_PER_METER = 1.98;
    public static final double KA_VOLTS_SECONDS_SQ_PER_METER = 0.2;
    public static final double P_DRIVE_VEL = 8.5;

    public static final double TRACKWIDTH_METERS = 0.69;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
        new DifferentialDriveKinematics(TRACKWIDTH_METERS);

    public static final double ENCODER_CONVERSION_FACTOR = 1; //find out experimentally or some other way

    public static final double TURN_CONSTANT = 0.015;
    public static final double ANGLE_DELTA = 1;
  }

  public static class TrajectoryConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
    
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }

}
