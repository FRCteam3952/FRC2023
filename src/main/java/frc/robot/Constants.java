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
    public static class ArmConstants {

        // All constants are in inches
        public static final double ORIGIN_HEIGHT = 42.0;
        public static final double LIMB1_LENGTH = 32.5;
        public static final double LIMB2_LENGTH = 19.5;

        public static final double ANGLE_DELTA = 2;
        public static final double DISTANCE_DELTA = 3;
        public static final double MIN_HOR_DISTANCE = 5; // change to be correct later

        public static final double PIVOT_SPEED = 0.1;
        public static final double TURRET_SPEED = 0.1;

        public static final double STARTING_X = 0;
        public static final double STARTING_Y = 0;
        public static final double STARTING_Z = 0;

        public static final double PICK_UP_POSITION_Y = 5;

        public static final double ARM_1_INITIAL_ANGLE = 10.0;
        public static final double ARM_2_INITIAL_ANGLE = 20.0;
    }

    public static class ClawConstants {
        public static final double CLAW_GRIP_SPEED = 0.2; // change to what this actually is
        public static final double CLAW_GRIP_SPEED_PASSIVE = 0.1;
        public static final double MAX_GRIP_ENCODER_VALUE = 3; // change to what this actually is
        public static final double MIN_GRIP_ENCODER_VALUE = 0; //change to what this actually is
        public static final double CLAW_ROTATE_SPEED = 0.1;
        public static final double ANGLE_DELTA = 2;
        public static final double MIN_ROTATION_ENCODER_VALUE = 0; // change to what it actually is
        public static final double MAX_ROTATION_ENCODER_VALUE = 3; // change to what it actually is
    }

    public static class OperatorConstants {
        public static final int RIGHT_JOYSTICK_PORT = 0;
        public static final int LEFT_JOYSTICK_PORT = 1;
    }

    public static class ControllerConstants {
        // Joystick 0
        public static final int RUN_GUI_TRAJECTORY_BUTTON_NUMBER = 4; // idk i just picked a random number
        // Joystick 1
        public static final int AIM_ASSIST_BUTTON_NUMBER = 5; // change to whatever it is
        public static final int MOVE_ARM_UP_BUTTON_NUMBER = 3; // change to whatever it is
        public static final int MOVE_ARM_DOWN_BUTTON_NUMBER = 2; // change to whatever it is
        public static final int MOVE_ARM_TO_PICK_UP_POSITION_BUTTON_NUMBER_FLIPPED = 11; // change to whatever it is
        public static final int MOVE_ARM_TO_PICK_UP_POSITION_BUTTON_NUMBER_NOT_FLIPPED = 12; // change to whatever it is
        public static final int CLAW_GRIP_BUTTON_NUMBER = 1; // change to whatever it is
        public static final int CLAW_RELEASE_BUTTON_NUMBER = 4;
        public static final int CLAW_ROTATE_RIGHT_BUTTON_NUMBER = 6; // change to whatever it is
        public static final int CLAW_ROTATE_LEFT_BUTTON_NUMBER = 7; // change to whatever it is
        public static final int AUTO_ROTATE_BUTTON_NUMBER = 8; // change to whatever it is
        public static final int CALIBRATE_ARM_BUTTON_NUMBER = 9; // change to whatever it is
        public static final int PID_CONTROL_TOGGLE_BUTTON_NUMBER = 10; // change to whatever it is
    }

    public static class PortConstants {
        public static final int FRONT_RIGHT_MOTOR_PORT = 1;
        public static final int REAR_RIGHT_MOTOR_PORT = 3;
        public static final int REAR_LEFT_MOTOR_PORT = 4;
        public static final int FRONT_LEFT_MOTOR_PORT = 2;

        public static final int PIVOT1_PORT = 9;
        public static final int PIVOT2_PORT = 10;
        public static final int TURRET_PORT = 8;

        public static final int CLAW_GRIP_PORT = 7;
        public static final int CLAW_ROTATE_PORT = 11;

        public static final int PIVOT_1_LIMIT_PORT = 1;
        public static final int PIVOT_2_LIMIT_PORT = 0;
    }

    public static class DriveConstants {// All constants below are examples; must correct to robot's specification when it is finished being built
        public static final double KS_VOLTS = 0.22;
        public static final double KV_VOLTS_SECONDS_PER_METER = 1.98;
        public static final double KA_VOLTS_SECONDS_SQ_PER_METER = 0.2;
        public static final double P_DRIVE_VEL = 8.5;

        public static final double TRACKWIDTH_METERS = 0.69; // nice
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

    public static class PositionConstants { // current coordinates are placeholders, replace with actual coordinates (xyz in inches)

        // https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/TeamVersions/Drawings/TE-23001-Grid-Dwg.pdf

        public static final double ROBOT_LENGTH = 27.0;

        public static final double Y_MIDDLE_POLE_HEIGHT = 34.0;
        public static final double Y_TOP_POLE_HEIGHT = 46.0;// ☻

        public static final double Y_MIDDLE_PLATFORM_HEIGHT = 33.5;
        public static final double Y_TOP_PLATFORM_HEIGHT = 35.5;

        public static final double Y_FLOOR = 0.0;

        public static final double Z_MIDDLE_DISTANCE = 22.75 + (ROBOT_LENGTH / 2);
        public static final double Z_TOP_DISTANCE = 39.75 + (ROBOT_LENGTH / 2);
        public static final double Z_BOTTOM_DISTANCE = 8.75 + (ROBOT_LENGTH / 2); // APPROXIMATION, TODO get better value & test

        public static final double X_BETWEEN_AREA_SPACE = 3.47;
        public static final double X_AREA_WIDTH = 18.5;

        // see top PDF to explain this
        // two halves of X_AREA_WIDTH added together and then added to X_BETWEEN_AREA_SPACE
        public static final double X_DISTANCE_TO_SIDE = X_BETWEEN_AREA_SPACE + X_AREA_WIDTH;

        public static final double X_LEFT = -X_DISTANCE_TO_SIDE;
        public static final double X_MIDDLE = 0.0;
        public static final double X_RIGHT = X_DISTANCE_TO_SIDE;

        // relative to the center of the robot
        public static final double[] BOTTOM_LEFT_POS = {X_LEFT, Y_FLOOR, Z_BOTTOM_DISTANCE};
        public static final double[] BOTTOM_MIDDLE_POS = {X_MIDDLE, Y_FLOOR, Z_BOTTOM_DISTANCE};
        public static final double[] BOTTOM_RIGHT_POS = {X_RIGHT, Y_FLOOR, Z_BOTTOM_DISTANCE};

        public static final double[] CENTER_LEFT_POS = {X_LEFT, Y_MIDDLE_POLE_HEIGHT, Z_MIDDLE_DISTANCE};
        public static final double[] CENTER_MIDDLE_POS = {X_MIDDLE, Y_MIDDLE_PLATFORM_HEIGHT, Z_MIDDLE_DISTANCE};
        public static final double[] CENTER_RIGHT_POS = {X_RIGHT, Y_MIDDLE_POLE_HEIGHT, Z_MIDDLE_DISTANCE};

        public static final double[] TOP_LEFT_POS = {X_LEFT, Y_TOP_POLE_HEIGHT, Z_TOP_DISTANCE};
        public static final double[] TOP_CENTER_POS = {X_MIDDLE, Y_TOP_PLATFORM_HEIGHT, Z_TOP_DISTANCE};
        public static final double[] TOP_RIGHT_POS = {X_RIGHT, Y_TOP_POLE_HEIGHT, Z_TOP_DISTANCE};


        // public static final Pose3d BOTTOM_LEFT_POS   = new Pose3d(X_LEFT,   Y_FLOOR, Z_BOTTOM_DISTANCE, new Rotation3d(0.0, 0.0, 0.0));
        // public static final Pose3d BOTTOM_MIDDLE_POS = new Pose3d(X_MIDDLE, Y_FLOOR, Z_BOTTOM_DISTANCE, new Rotation3d(0.0, 0.0, 0.0));
        // public static final Pose3d BOTTOM_RIGHT_POS  = new Pose3d(X_RIGHT,  Y_FLOOR, Z_BOTTOM_DISTANCE, new Rotation3d(0.0, 0.0, 0.0));

        // public static final Pose3d CENTER_LEFT_POS   = new Pose3d(X_LEFT,   Y_MIDDLE_POLE_HEIGHT,     Z_MIDDLE_DISTANCE, new Rotation3d(0.0, 0.0, 0.0));
        // public static final Pose3d CENTER_MIDDLE_POS = new Pose3d(X_MIDDLE, Y_MIDDLE_PLATFORM_HEIGHT, Z_MIDDLE_DISTANCE, new Rotation3d(0.0, 0.0, 0.0));
        // public static final Pose3d CENTER_RIGHT_POS  = new Pose3d(X_RIGHT,  Y_MIDDLE_POLE_HEIGHT,     Z_MIDDLE_DISTANCE, new Rotation3d(0.0, 0.0, 0.0));

        // public static final Pose3d TOP_LEFT_POS      = new Pose3d(X_LEFT,   Y_TOP_POLE_HEIGHT,     Z_TOP_DISTANCE, new Rotation3d(0.0, 0.0, 0.0));
        // public static final Pose3d TOP_CENTER_POS    = new Pose3d(X_MIDDLE, Y_TOP_PLATFORM_HEIGHT, Z_TOP_DISTANCE, new Rotation3d(0.0, 0.0, 0.0));
        // public static final Pose3d TOP_RIGHT_POS     = new Pose3d(X_RIGHT,  Y_TOP_POLE_HEIGHT,     Z_TOP_DISTANCE, new Rotation3d(0.0, 0.0, 0.0));

    }

    public static class AprilTagConstants {
        /**
         * <pre>
         * All distances are in inches, located on pg 4 here: <a href="https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf">...</a>, to the center of the tag.
         * Relative to origin at right corner from Blue Alliance perspective.
         * +Y is towards the left corner,
         * +X is towards the Red Alliance.
         *
         * Index 0 is TagID 1, Index 1 is TagID 2, etc.
         * </pre>
         */
        public static final double[][] tagInfo = new double[][]{
                {610.77, 42.19, 18.22, 180},
                {610.77, 108.19, 18.22, 180},
                {610.77, 174.19, 18.22, 180},
                {636.96, 265.74, 27.38, 180},
                {14.25, 265.74, 27.38, 0},
                {40.45, 174.19, 18.22, 0},
                {40.45, 108.19, 18.22, 0},
                {40.45, 42.19, 18.22, 0}
        };
    }
}
