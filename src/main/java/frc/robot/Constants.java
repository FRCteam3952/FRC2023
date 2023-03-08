// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.util.ForwardKinematicsUtil;
import frc.robot.util.InverseKinematicsUtil;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@SuppressWarnings("unused")
public final class Constants {
    /**
     * Constants for the Arm
     */
    public static class ArmConstants {

        // All constants are in inches
        public static final double ORIGIN_HEIGHT = 42d;
        public static final double LIMB1_LENGTH = 32.5;
        public static final double LIMB2_LENGTH = 29d;

        public static final double ANGLE_DELTA = 5.0;
        public static final double MAX_REACH_REDUCTION = 0; // change to be correct later
        public static final double MIN_HOR_DISTANCE = 6; // change to be correct later

        public static final double PIVOT_SPEED = 0.1;
        public static final double TURRET_SPEED = 0.1;

        public static final double STARTING_X = 0;
        public static final double STARTING_Y = 0;
        public static final double STARTING_Z = 0;

        public static final double PICK_UP_POSITION_Y = 5;

        public static final double PID_TOLERANCE = 1.0;

        public static final double ARM_1_INITIAL_ANGLE = 10.0;
        public static final double ARM_2_INITIAL_ANGLE = 20.0;
        public static final double MAX_OUTPUT = 0.4;
        public static final double MIN_OUTPUT = -0.4;
        public static final double SPEED_DEC_ON_UNFLIP = 0.2;
        public static final double SPEED_DEC_ON_FLIP = 1.0;
        public static final double COMPLEMENTING_FLIP_SPEED = 1.2;
        public static final double[] STARTING_COORDS = ForwardKinematicsUtil.getCoordinatesFromAngles(ARM_1_INITIAL_ANGLE, ARM_2_INITIAL_ANGLE, 0);
        public static final double[] FLIP_COORDS_WHEN_FLIPPING = {31.0, 17.0, 0.0};
        public static final double[] FLIPPING_TARGET_ANGLES = InverseKinematicsUtil.getAnglesFromCoordinates(FLIP_COORDS_WHEN_FLIPPING[0], FLIP_COORDS_WHEN_FLIPPING[1], FLIP_COORDS_WHEN_FLIPPING[2], true);
    }

    /**
     * Constants for the Claw
     */
    public static class ClawConstants {
        public static final double CLAW_ROTATE_SPEED = 0.1;
        public static final double ANGLE_DELTA = 2;
        public static final double MIN_ROTATION_ENCODER_VALUE = 0; // change to what it actually is
        public static final double MAX_ROTATION_ENCODER_VALUE = 3; // change to what it actually is
    }

    /**
     * Constants relating to the Drive Station (joysticks)
     */
    public static class OperatorConstants {
        public static final int RIGHT_JOYSTICK_PORT = 0;
        public static final int LEFT_JOYSTICK_PORT = 1;
        public static final int XBOX_CONTROLLER_PORT = 2;

        /**
         * Constants relating to the controllers.
         * The designation of which buttons go on which controller are subject to future change.
         */
        public static class ControllerConstants {
            // Flight Joystick 0
            public static final int RUN_GUI_TRAJECTORY_BUTTON_NUMBER = 4; // idk i just picked a random number

            // Xbox Controller
            public static final int AIM_ASSIST_BUTTON_NUMBER = 1; // change to whatever it is
            public static final int EXTEND_ARM_BUTTON_NUMBER = 3; // change to up arrow on d-pad
            public static final int RETRACT_ARM_BUTTON_NUMBER = 2; // change to down arrow on d-pad
            public static final int FLIP_ARM_BUTTON_NUMBER = 5;
            /**
             *  THIS IS THE XBOX NUMBER. Flight joystick original number: 1;
             */
            public static final int CLAW_GRIP_OR_RELEASE_BUTTON_NUMBER = 6; // THIS IS THE XBOX NUMBER. Flight joystick: 1;
            public static final int AUTO_ROTATE_BUTTON_NUMBER = 8; // change to whatever it is
            public static final int CALIBRATE_ARM_BUTTON_NUMBER = 9; // change to whatever it is
            public static final int MOVE_ARM_UP_BUTTON_NUMBER = 10;

            // Xbox Controller (2)

        }
    }

    /**
     * Constants relating to ports on the robot (which port things are plugged into)
     */
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
        public static final int CLAW_LIMIT_SWITCH_PORT = 2; // change to whatever it is
    }

    /**
     * Constants relating to the drive train
     */
    public static class DriveConstants {// All constants below are examples; must correct to robot's specification when it is finished being built
        public static final double KS_VOLTS = 0.22;
        public static final double KV_VOLTS_SECONDS_PER_METER = 1.98;
        public static final double KA_VOLTS_SECONDS_SQ_PER_METER = 0.2;
        public static final double P_DRIVE_VEL = 8.5;

        public static final double TRACKWIDTH_METERS = 0.69; // nice
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACKWIDTH_METERS);

        public static final double ENCODER_CONVERSION_FACTOR = 1; //find out experimentally or some other way

        public static final double TURN_CONSTANT = 0.02; // was 0.015
        public static final double ANGLE_DELTA = 1;

        /**
         * Constants relating to trajectory generation
         */
        public static class TrajectoryConstants {
            public static final double MAX_SPEED_METERS_PER_SECOND = 3;
            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

            // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
            public static final double RAMSETE_B = 2;
            public static final double RAMSETE_ZETA = 0.7;
        }
    }

    public static class PositionConstants { // current coordinates are placeholders, replace with actual coordinates (xyz in inches)
        // https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/TeamVersions/Drawings/TE-23001-Grid-Dwg.pdf

        public static final double ROBOT_LENGTH = 27.0;

        public static final double Y_MIDDLE_POLE_HEIGHT = 34.0;
        public static final double Y_TOP_POLE_HEIGHT = 46.0;

        public static final double Y_MIDDLE_PLATFORM_HEIGHT = 23.5;
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
    }

    /**
     * All robot-related constants. All measurements are in inches
     */
    public static class RobotConstants {
        /*
         * One of these is 30, and one is 35. I don't know which is which and honestly just use the diagonal distance.
         */
        public static final double ROBOT_WIDTH = 30.0;
        public static final double ROBOT_LENGTH = 35.0;
        public static final double ROBOT_DIAGONAL_RADIUS = Math.sqrt(Math.pow(ROBOT_WIDTH / 2, 2) + Math.pow(ROBOT_LENGTH / 2, 2)); // about 23.05 inches
    }

    /**
     * All field-related constants. All measurements are in inches
     */
    public static class FieldConstants {
        public static final double FIELD_Y_LENGTH = 315.5;
        public static final double FIELD_X_LENGTH = 651.25;

        /**
         * {@link RobotPlacementLocations#MINIMUM_X_LOCATION_TO_PLACE_FROM} represents the minimum X value (in a pose) that the robot can place a gamepiece from safely (i.e. not colliding into the structure).
         * <p>
         * The Y value is the same as the corresponding target game piece location (ex. {@link GamePiecePlacementLocationConstants#Y_DISTANCE_TO_POLE_ONE})
         */
        public static class RobotPlacementLocations {
            public static final double DISTANCE_FROM_LOWER_POLE_TO_FAR_EDGE_OF_FLOOR_NODE = 25;
            public static final double MINIMUM_X_LOCATION_TO_PLACE_FROM = DISTANCE_FROM_LOWER_POLE_TO_FAR_EDGE_OF_FLOOR_NODE + RobotConstants.ROBOT_DIAGONAL_RADIUS;
        }

        /**
         * These values were obtained from Solidworks with the measurement tool. It took me an hour to do this (and another 4 to set up solidworks correctly) so they better work.
         * <p>
         * The original values are for the blue alliance (since the origin is on the blue alliance's side), so we use {@link frc.robot.util.MathUtil#mirrorValueOnFieldForOppositeSide(double) MathUtil.mirrorValueOnFieldXAxis(double)} to mirror the values for the red alliance.
         * <p>
         * The Z values were acquired by Ivan, and are found on page 27 (lower image) of the FRC 2023 Game Manual.
         * <p>
         * From the pinned message in Discord:
         * <pre>
         * PLACEMENT LOCATIONS based on AprilTags-defined coordinate system, where "closest" means the closest to the origin.
         * ALL IN INCHES:
         *
         * POLES (Y axis):
         *   dist to closest (1): 20.32
         *        to next:        64.33
         *        to next:        86.19
         *        to next:        130.33
         *        to next:        152.47
         *        to last    (6): 196.36
         *
         * CUBE PLATFORMS (Y axis):
         *   dist to closest (1): 42.48
         *        to next:        108.5
         *        to last    (3): 174.45
         *
         * Top, Middle, Bottom (X axis):
         *   top: 15
         *   mid: 32
         *   low: 47
         * </pre>
         * 
         * The layout would look like this:
         * 
         * <pre>
         * BLUE SIDE
         * Origin -> +Y
         * |  Top Z:   Pole1 Plat1 Pole2 Pole3 Plat2 Pole4 Pole5 Plat3 Pole6
         * v  Mid Z:   Pole1 Plat1 Pole2 Pole3 Plat2 Pole4 Pole5 Plat3 Pole6
         * +X Low (0): Pole1 Plat1 Pole2 Pole3 Plat2 Pole4 Pole5 Plat3 Pole6
         * 
         * </pre>
         * This image would be reflected across the middle of the field, on a line parallel to the Y axis for the Red alliance's side.
         */
        public static class GamePiecePlacementLocationConstants {
            public static final double Y_DISTANCE_TO_POLE_ONE       = 20.32;
            public static final double Y_DISTANCE_TO_POLE_TWO       = 64.33;
            public static final double Y_DISTANCE_TO_POLE_THREE     = 86.19;
            public static final double Y_DISTANCE_TO_POLE_FOUR      = 130.33;
            public static final double Y_DISTANCE_TO_POLE_FIVE      = 152.47;
            public static final double Y_DISTANCE_TO_POLE_SIX       = 196.36;

            public static final double Y_DISTANCE_TO_PLATFORM_ONE   = 42.48;
            public static final double Y_DISTANCE_TO_PLATFORM_TWO   = 108.5;
            public static final double Y_DISTANCE_TO_PLATFORM_THREE = 174.45;

            public static final double X_DISTANCE_TO_TOP_SECTION    = 15.0;
            public static final double X_DISTANCE_TO_MIDDLE_SECTION = 32.0;
            public static final double X_DISTANCE_TO_BOTTOM_SECTION = 47.0;

            public static final double Z_MIDDLE_POLE_HEIGHT         = 34.0;
            public static final double Z_TOP_POLE_HEIGHT            = 46.0;

            public static final double Z_MIDDLE_PLATFORM_HEIGHT     = 23.5;
            public static final double Z_TOP_PLATFORM_HEIGHT        = 35.5;

            public static final Pose3d[][] POLE_POSITIONS = new Pose3d[3][6];
            public static final Pose3d[][] PLATFORM_POSITIONS = new Pose3d[3][3];

            private static final double[] X_COORDS = new double[] { // len3 all
                X_DISTANCE_TO_TOP_SECTION, X_DISTANCE_TO_MIDDLE_SECTION, X_DISTANCE_TO_BOTTOM_SECTION
            };

            private static final double[] POLE_Y_COORDS = new double[] { // len6 pole
                Y_DISTANCE_TO_POLE_ONE, Y_DISTANCE_TO_POLE_TWO, Y_DISTANCE_TO_POLE_THREE, Y_DISTANCE_TO_POLE_FOUR, Y_DISTANCE_TO_POLE_FIVE, Y_DISTANCE_TO_POLE_SIX
            };

            private static final double[] POLE_Z_COORDS = new double[] { // len3 pole vertical
                Z_TOP_POLE_HEIGHT, Z_MIDDLE_POLE_HEIGHT, 0.0
            };

            private static final double[] PLATFORM_Y_COORDS = new double[] { // len3 platform
                Y_DISTANCE_TO_PLATFORM_ONE, Y_DISTANCE_TO_PLATFORM_TWO, Y_DISTANCE_TO_PLATFORM_THREE
            };

            private static final double[] PLATFORM_Z_COORDS = new double[] { // len3 platform vertical
                Z_TOP_PLATFORM_HEIGHT, Z_MIDDLE_PLATFORM_HEIGHT, 0.0
            };

            static {
                for(int i = 0; i < 3; i++) { // POLE
                    for(int j = 0; j < 5; j++) {
                        POLE_POSITIONS[i][j] = new Pose3d(X_COORDS[i], POLE_Y_COORDS[j], POLE_Z_COORDS[i], new Rotation3d());
                    }
                }

                for(int i = 0; i < 3; i++) { // PLATFORM
                    for(int j = 0; j < 3; j++) {
                        PLATFORM_POSITIONS[i][j] = new Pose3d(X_COORDS[i], PLATFORM_Y_COORDS[j], PLATFORM_Z_COORDS[i], new Rotation3d());
                    }
                }
            }
        }

        public static class AprilTagConstants {
            /**
             * <pre>
             * All distances are in inches, located on pg 4 here: <a href="https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf">...</a>, to the center of the tag.
             * Relative to origin at right corner from Blue Alliance perspective.
             * +Y is towards the left corner,
             * +X is towards the Red Alliance.
             *
             * Stored as {x, y, z, angle}, though angle is irrelevant for us.
             *
             * Index 0 is TagID 1, Index 1 is TagID 2, etc.
             * </pre>
             */
            public static final double[][] APRILTAG_LOCATIONS = new double[][]{
                    {610.77, 42.19,  18.22, 180},
                    {610.77, 108.19, 18.22, 180},
                    {610.77, 174.19, 18.22, 180},
                    {636.96, 265.74, 27.38, 180},
                    {14.25,  265.74, 27.38, 0},
                    {40.45,  174.19, 18.22, 0},
                    {40.45,  108.19, 18.22, 0},
                    {40.45,  42.19,  18.22, 0}
            };
        }
    }
}
