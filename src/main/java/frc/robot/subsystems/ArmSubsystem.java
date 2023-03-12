package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.armcommands.FlipArmCommand;
import frc.robot.util.ForwardKinematicsUtil;
import frc.robot.util.InverseKinematicsUtil;
import frc.robot.util.MathUtil;

/*
 * Arm axis control scheme:
 * 
 * Top down view:
 *                           x-axis
 *                         0 degrees
 *                             |
 *                             |
 *  z-axis  270 degrees --------------- 90 degrees
 *                             |
 *                             |
 *                        180 degrees
 *  
 * y-axis is the vertical axis (perpendicular to x and z axes)
 *
 */

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax pivot1;
    private final CANSparkMax pivot2;
    private final CANSparkMax turret;

    private final RelativeEncoder pivot1Encoder;
    private final RelativeEncoder pivot2Encoder;
    private final RelativeEncoder turretEncoder;

    private final DigitalInput arm1Limit;
    private final DigitalInput arm2Limit;
    private final DigitalInput turretLimit;

    private final PIDController pidController1, pidController2, pidController3;

    private double targetX;
    private double targetY;
    private double targetZ;

    private double cur_x;
    private double cur_y;
    private double cur_z;

    private double targetAngle1;
    private double targetAngle2;
    private double targetAngleTurret;

    private boolean pidOn = false;
    private boolean flipped = false;

    private double arm1SpeedMultiplier = 1;
    private double arm2SpeedMultiplier = 1;

    private boolean isManual = true;
    private boolean is2D = true;

    private double maxOutput = ArmConstants.MAX_OUTPUT;
    private double minOutput = ArmConstants.MIN_OUTPUT;
    private double maxOutput2 = ArmConstants.MAX_OUTPUT;
    private double minOutput2 = ArmConstants.MIN_OUTPUT;

    // arm control constructor
    public ArmSubsystem() {
        // Initialize arm motors
        this.pivot1 = new CANSparkMax(PortConstants.PIVOT1_PORT, MotorType.kBrushless);
        this.pivot2 = new CANSparkMax(PortConstants.PIVOT2_PORT, MotorType.kBrushless);
        this.turret = new CANSparkMax(PortConstants.TURRET_PORT, MotorType.kBrushless);

        this.pivot1.setInverted(true);
        this.pivot2.setInverted(true);
        this.turret.setInverted(false);

        // Set up arm encoders and position conversion factors
        this.pivot1Encoder = this.pivot1.getEncoder();
        this.pivot2Encoder = this.pivot2.getEncoder();
        this.turretEncoder = this.turret.getEncoder();

        // Arm Angle Conversion Factors
        this.pivot1Encoder.setPositionConversionFactor(2.7); // 125:1 gearbox
        this.pivot2Encoder.setPositionConversionFactor(3.65); // 125:1 gearbox
        this.turretEncoder.setPositionConversionFactor(1); // 60:1 gearbox with drive wheel to lazy susan ratio
        // END

        this.pivot1Encoder.setPosition(ArmConstants.ARM_1_INITIAL_ANGLE);
        this.pivot2Encoder.setPosition(ArmConstants.ARM_2_INITIAL_ANGLE);
        this.turretEncoder.setPosition(0);
        // TODO: TUNE
        this.pidController1 = new PIDController(1.6e-2, 0, 0); // nice
        this.pidController1.setTolerance(ArmConstants.PID_TOLERANCE);
        this.pidController2 = new PIDController(1.6e-2, 0, 0);
        this.pidController2.setTolerance(ArmConstants.PID_TOLERANCE);
        this.pidController3 = new PIDController(1e-2, 0, 0);
        this.pidController3.setTolerance(ArmConstants.PID_TOLERANCE);
        // END

        // Initialize arm limit switches
        this.arm1Limit = new DigitalInput(PortConstants.PIVOT_1_LIMIT_PORT);
        this.arm2Limit = new DigitalInput(PortConstants.PIVOT_2_LIMIT_PORT);
        this.turretLimit = new DigitalInput(PortConstants.TURRET_LIMIT_PORT);

        // Set starting arm angles
        this.targetAngle1 = ArmConstants.ARM_1_INITIAL_ANGLE;
        this.targetAngle2 = ArmConstants.ARM_2_INITIAL_ANGLE;
        this.targetAngleTurret = 0;

        // Get starting coords from the initial angle constants
        resetCoords();
    }

    /**
     * Sets the turret direction. Must be 1 or -1.
     * @param dir The direction. This acts as a sign multiplier for the turret to flip the direction as needed.
     */

    /**
     * Gets the turret direction
     * @return The turret direction. Should be 1 or -1 (unless someone trolled us)
     */

    public void reset() {
        this.pidOn = false;
        resetCoords();
    }
    public void resetCoords() {
        this.targetX = ArmConstants.STARTING_COORDS[0];
        this.targetY = ArmConstants.STARTING_COORDS[1];
        this.targetZ = ArmConstants.STARTING_COORDS[2];
        this.cur_x = ArmConstants.STARTING_COORDS[0];
        this.cur_y = ArmConstants.STARTING_COORDS[1];
        this.cur_z = ArmConstants.STARTING_COORDS[2];
        this.targetAngle1 = ArmConstants.ARM_1_INITIAL_ANGLE;
        this.targetAngle2 = ArmConstants.ARM_2_INITIAL_ANGLE;
        this.pivot1Encoder.setPosition(ArmConstants.ARM_1_INITIAL_ANGLE);
        this.pivot2Encoder.setPosition(ArmConstants.ARM_2_INITIAL_ANGLE);
    }

    public void setMaxAndMinOutput1(double speed) {
        this.minOutput = -speed;
        this.maxOutput = speed;
    }
    public void setMaxAndMinOutput2(double speed){
        this.minOutput2 = -speed;
        this.maxOutput2 = speed;
    }

    public double resetTurretEncoder() {
        return this.turretEncoder.getPosition();
    }

    public PIDController getPID1() {
        return this.pidController1;
    }

    public PIDController getPID2() {
        return this.pidController2;
    }


    /**
     * Changes the intended coordinates by dx, dy, and dz
     */
    public void moveVector(double dx, double dy, double dz) {
        updateCurrentCoordinates();
        setTargetCoordinates(targetX + dx, targetY + dy, targetZ + dz);
    }

    /**
     * Get the current angles from motor encoders in DEGREES
     * 
     * @return [pivot1Angle, pivot2Angle, turretAngle]
     */
    public double[] getCurrentAnglesDeg() {
        double angle1 = pivot1Encoder.getPosition();
        //double angle2 = (90 + angle1 + (MPU6050.getRoll()-80));
        double angle2 = pivot2Encoder.getPosition();
        double angle3 = turretEncoder.getPosition();

        if (flipped) { //offset for when arm is flipped because our gearbox is lose for some reason
            angle1 -= 8;
        }

        return new double[]{angle1, angle2, angle3};
    }

    /**
     * Get the current angles from motor encoders in radians
     * 
     * @return [pivot1Angle, pivot2Angle, turretAngle]
     */
    public double[] getCurrentAnglesRad() {
        double angle1 = Math.toRadians(pivot1Encoder.getPosition());
        //double angle2 = Math.toRadians((90 + angle1 + (MPU6050.getRoll()-80)));
        double angle2 = pivot2Encoder.getPosition();
        double angle3 = Math.toRadians(turretEncoder.getPosition());

        return new double[]{angle1, angle2, angle3};
    }

    public double getTurretAngleDeg() {
        return this.turretEncoder.getPosition();
    }


    public void setPivot1Speed(double speed) {
        this.pivot1.set(speed);
    }

    public void setPivot2Speed(double speed) {
        this.pivot2.set(speed);
    }

    public void setTurretSpeed(double speed) {
        this.turret.set(speed);

    }

    public void setManualControlMode(boolean isManual){
        this.isManual = isManual;
    }

    public void setControlDimensions(boolean is2D){
        this.is2D = is2D;
    }

    public void stopAllMotors() {
        this.pivot1.set(0);
        this.pivot2.set(0);
        this.turret.set(0);
    }

    public void setTargetAngle1(double angle) {
        this.targetAngle1 = angle;
    }

    public void setTargetAngle2(double angle) {
        this.targetAngle2 = angle;
    }

    public boolean getControlMode(){
        return isManual;
    }

    /**
     * Uses motor encoder angles to update the current coordinates
     */
    public void updateCurrentCoordinates() {
        double[] tempAngles = getCurrentAnglesDeg();
        double[] coords = ForwardKinematicsUtil.getCoordinatesFromAngles(tempAngles[0], tempAngles[1], tempAngles[2]);
        this.cur_x = coords[0];
        this.cur_y = coords[1];
        this.cur_z = coords[2];
    }

    /*
     * returns current coordinates
     * 
     * @return [x, y, z], where y is the height above ground.
     */
    public double[] getCurrentCoordinates() {
        updateCurrentCoordinates();
        return new double[]{this.cur_x, this.cur_y, this.cur_z};
    }

    /**
     * return coordinates in which the arm "should" move towards
     * 
     * @return [x, y, z], where y is the height above ground
     */
    public double[] getTargetCoordinates() {
        return new double[]{this.targetX, this.targetY, this.targetZ};
    }

    public boolean getPivot1LimitPressed() {
        return !this.arm1Limit.get();
    }

    public boolean getPivot2LimitPressed() {
        return !this.arm2Limit.get();
    }

    public boolean getTurretLimitPressed(){
        return !this.turretLimit.get();
    }

    public void setArm1SpeedMultiplier(double mult) {
        this.arm1SpeedMultiplier = mult;
    }

    public void setArm2SpeedMultiplier(double mult) {
        this.arm2SpeedMultiplier = mult;
    }

    public void goTowardTargetCoordinates() {
        double[] angles = getCurrentAnglesDeg(); // gets the current angles read from motor encoders

        if (Double.isNaN(angles[0]) || Double.isNaN(angles[1]) || Double.isNaN(angles[2]) || Double.isNaN(targetAngle1) || Double.isNaN(targetAngle2) || Double.isNaN(targetAngleTurret)) {
            System.out.println("An angle is NaN, so skip");
            return;
        }

        // gets PID control calculations
        double p1Speed = pidController1.calculate(angles[0], targetAngle1) * arm1SpeedMultiplier;
        double p2Speed = pidController2.calculate(angles[1], targetAngle2) * arm2SpeedMultiplier;
        double turretSpeed = 0;

        // if power is NaN, don't run it :D
        if (Double.isNaN(p1Speed) || Double.isNaN(p2Speed)) {
            System.out.println("PID is NaN, so skip");
            return;
        }

        p1Speed = Math.min(maxOutput, Math.max(p1Speed, minOutput));
        p2Speed = Math.min(maxOutput2, Math.max(p2Speed, minOutput2));

        if (!is2D) { //only control turret or Z axis when auto
            turretSpeed = pidController3.calculate(angles[2], targetAngleTurret);
            turretSpeed = Math.min(maxOutput, Math.max(turretSpeed, minOutput));
            if(Double.isNaN(turretSpeed)){
                return;
            }
            setTurretSpeed(turretSpeed);
        }
        setPivot1Speed(p1Speed);
        setPivot2Speed(p2Speed);

        // System.out.println("SPEEDS: " + p1Speed + " " + p2Speed + " " + turretSpeed);
    }

    /**
     * sets the coordinate in which the arm "should" move towards
     * 
     * @param x the target x coordinate
     * @param y the target y coordinate (height)
     * @param z the target z coordinate
     * @param flipped whether the arm should act "flipped", i.e. the claw would approach from the gamepiece from the top. True for "top approach", False for "side approach"
     */
    public void setTargetCoordinates(double x, double y, double z) {
        if (this.targetX == x && this.targetY == y && this.targetZ == z) { // if intended coordinates are same, then don't change target
            return;
        }
        if(y > 75){
            y = 75;
        }

        if(this.is2D){
            z = 0;
        }

        // Updates target Angles
        double[] targetAngles = InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z, getFlipped());
    
        // Stops any updates if IKU is out of bounds or calculation error occurs
        if (Double.isNaN(targetAngles[0]) || Double.isNaN(targetAngles[1]) || Double.isNaN(targetAngles[2])) {
            System.out.println("Hi this is the Arm Death Prevention Hotline @copyright setIntendedCoordinates");
            return;
        }

        // Updates target angles
        targetAngle1 = targetAngles[0];
        targetAngle2 = targetAngles[1];
        targetAngleTurret = targetAngles[2];

        // Updates target coordinates
        double[] adjustedCoordinates = ForwardKinematicsUtil.getCoordinatesFromAngles(targetAngle1, targetAngle2, targetAngleTurret);
        this.targetX = adjustedCoordinates[0];
        this.targetY = adjustedCoordinates[1];
        this.targetZ = adjustedCoordinates[2];
    }

    public CommandBase calibrateArm() {
        pidOn = false;
        return this.runOnce(() -> {
            while (!getPivot2LimitPressed()) {
                setPivot2Speed(-0.2);
            }
            setPivot2Speed(0);
            while (!getPivot1LimitPressed()) {
                setPivot1Speed(-0.2);
            }
            setPivot1Speed(0);
            resetCoords();
            pidOn = true;
        });
    }

    /**
     * Sets the usage of PID control for the arm.
     * @param value True for enabled PID, False for disabled.
     */
    public void setPIDControlState(boolean value) {
        pidOn = value;
        if(!value) {
            stopAllMotors();
        }
    }

    /**
     * Get the usage of PID control for the arm.
     * @return True if enabled, false otherwise
     */
    public boolean getPIDControlOn() {
        return pidOn;
    }

    public boolean getFlipped(){
        return flipped;
    }

    public void setFlipped(boolean flipped){
        if(!flipped){
            moveVector(0, 10, 0);
        }
        this.flipped = flipped;
        (new FlipArmCommand(this, flipped)).withInterruptBehavior(InterruptionBehavior.kCancelSelf).schedule();
    }

    public boolean isAtCoords(){
        double[] curAngles = getCurrentAnglesDeg();
        return (Math.abs(targetAngle1 - curAngles[0]) < ArmConstants.ANGLE_DELTA) && (Math.abs(targetAngle2 - curAngles[1]) < ArmConstants.ANGLE_DELTA) && (Math.abs(targetAngleTurret - curAngles[2]) < ArmConstants.ANGLE_DELTA);
    }

    /**
     * Returns the current claw pose. Note that the claw pose is stored as {x, z, y}, where y is the height of the claw. This allows you to use {@link Pose3d#toPose2d()} to get a correct 2D pose.
     * @return A Pose3d object representing the current claw pose.
     */
    public Pose3d getClawPose() {
        this.updateCurrentCoordinates(); // Make sure the coordinates are the latest ones.
        return new Pose3d(MathUtil.inchesToMeters(this.cur_x), MathUtil.inchesToMeters(this.cur_z), MathUtil.inchesToMeters(this.cur_y), new Rotation3d()); // z and y are swapped to handle our global coordinate system (the final coord parameter is the height).
    }

    @Override
    public void periodic() {
        // System.out.println("ARM MOTOR ENCODERS: PIV1: " + this.pivot1Encoder.getPosition() + ", PIV2: " + this.pivot2Encoder.getPosition() + ", TURRET: " + this.turretEncoder.getPosition());
        // System.out.println("TARGET COORDS: " + targetX + ", " + targetY + ", " + targetZ);
        // System.out.println("ARM IKU FLIP STATE: " + this.flipped);
        // System.out.println("TARGET ANGLES: " + targetAngle1 + ", " + targetAngle2 + ", " + targetAngleTurret);
        // System.out.println("CURRENT ANGLES " + getCurrentAnglesDeg()[0] + " " + getCurrentAnglesDeg()[1] + " " + getCurrentAnglesDeg()[2]);
        boolean resetPivot1 = getPivot1LimitPressed() && Math.abs(this.pivot1Encoder.getPosition() - ArmConstants.ARM_1_INITIAL_ANGLE) > 0.1 && Math.abs(targetAngle1 - ArmConstants.ARM_1_INITIAL_ANGLE) < 5;
        boolean resetPivot2 = getPivot2LimitPressed() && Math.abs(this.pivot2Encoder.getPosition() - ArmConstants.ARM_2_INITIAL_ANGLE) > 0.1 && Math.abs(targetAngle2 - ArmConstants.ARM_2_INITIAL_ANGLE) < 5;

        double tempAngle = this.turretEncoder.getPosition();
        // System.out.println("TURRET SPEED: " + turret.get() + ", ANG: " + getTurretAngleDeg());
        
        if(getTurretLimitPressed()){
            //if(tempAngle < 180 && tempAngle > -180){
                //System.out.println("RESET TURRTE");
                this.turretEncoder.setPosition(0);
            //}
        }

        // System.out.println("LIMIT 1: " + getPivot1LimitPressed() + ", LIMIT 2: " + getPivot2LimitPressed());
        // System.out.println("Turret Limit: " + getTurretLimitPressed());
        // handles limit switches

        if (resetPivot1) {
            this.pivot1Encoder.setPosition(ArmConstants.ARM_1_INITIAL_ANGLE);
        }

        if (resetPivot2) {
            this.pivot2Encoder.setPosition(ArmConstants.ARM_2_INITIAL_ANGLE);
        }

        if(resetPivot1 && resetPivot2) {
            resetCoords();
        }

        


        /*
         * Moves arm to specific positions for placing game pieces. The robot is assumed to be positioned directly in front of an april tag.
         * Each key corresponds to a specific position on a 3x3 grid, as shown below:
         *          [7  8  9]
         *          [4  5  6]
         *          [1  2  3]
         *              ^
         *            robot
         * 
         * There are 3 of these 3x3 grids.
         * 
         * A check for a joystick button pressed or something similar can be added if we want to add another check other than the key on the keyboard being pressed
         */

        //TODO: fix - ebay kid, 3-1-2023. the constants are wrong
        /*
        double[] coordinates = new double[3];
        int currKey = NetworkTablesUtil.getKeyInteger();
        switch (currKey) {
            case 1:
                coordinates = PositionConstants.BOTTOM_LEFT_POS;
                setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2], false);
                break;
            case 2:
                coordinates = PositionConstants.BOTTOM_MIDDLE_POS;
                setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2], false);
                break;
            case 3:
                coordinates = PositionConstants.BOTTOM_RIGHT_POS;
                setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2], false);
                break;
            case 4:
                coordinates = PositionConstants.CENTER_LEFT_POS;
                setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2], false);
                break;
            case 5:
                coordinates = PositionConstants.CENTER_MIDDLE_POS;
                setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2], false);
                break;
            case 6:
                coordinates = PositionConstants.CENTER_RIGHT_POS;
                setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2], false);
                break;
            case 7:
                coordinates = PositionConstants.TOP_LEFT_POS;
                setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2], false);
                break;
            case 8:
                coordinates = PositionConstants.TOP_CENTER_POS;
                setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2], false);
                break;
            case 9:
                coordinates = PositionConstants.TOP_RIGHT_POS;
                setIntendedCoordinates(coordinates[0], coordinates[1], coordinates[2], false);
                break;
            default:
                System.out.println("A key within 1-9 was not pressed");
                break;
                
        }
        */
        // double[] angles = InverseKinematicsUtil.getAnglesFromCoordinates(30, ArmConstants.ORIGIN_HEIGHT, 30, false);
        // System.out.println(angles[0] + " " + angles[1] + " " + angles[2]);
        // double[] coords = ForwardKinematicsUtil.getCoordinatesFromAngles(angles[0], angles[1], angles[2]);
        // System.out.println(coords[0] + " " + coords[1] + " " + coords[2]);

        //handles PID
        // System.out.println("PID STATE: " + pidOn);
        if (pidOn) {
            goTowardTargetCoordinates();
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}