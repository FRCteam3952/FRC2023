package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.controllers.XboxController;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.armcommands.FlipArmCommand;
import frc.robot.commands.armcommands.GoTowardsCoordinatesCommandTeleop;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.util.ForwardKinematicsUtil;
import frc.robot.util.InverseKinematicsUtil;
import frc.robot.util.MathUtil;
import frc.robot.util.NetworkTablesUtil;

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


    private final DutyCycleEncoder arm1AbsoluteEncoder;
    private final DutyCycleEncoder arm2AbsoluteEncoder;

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

    private boolean isAtHumanPlayer = false;

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
        this.turretEncoder.setPositionConversionFactor(0.64); // 125:1 gearbox with drive wheel to lazy susan ratio
        // END

        this.arm1AbsoluteEncoder = new DutyCycleEncoder(PortConstants.PIVOT_1_ABSOLUTE_ENCDOER_PORT);
        this.arm2AbsoluteEncoder = new DutyCycleEncoder(PortConstants.PIVOT_2_ABSOLUTE_ENCDOER_PORT);

        this.pivot1Encoder.setPosition(ArmConstants.ARM_1_INITIAL_ANGLE);
        this.pivot2Encoder.setPosition(ArmConstants.ARM_2_INITIAL_ANGLE);
        this.turretEncoder.setPosition(0);
        // TODO: TUNE
        this.pidController1 = new PIDController(1.8e-2, 0, 0); // nice
        this.pidController1.setTolerance(ArmConstants.PID_TOLERANCE);
        this.pidController2 = new PIDController(1.6e-2, 0, 0);
        this.pidController2.setTolerance(ArmConstants.PID_TOLERANCE);
        this.pidController3 = new PIDController(2.8e-2, 0, 0);
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
        correctMotorEncoders();
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
        this.targetAngleTurret = 0;
    }

    public void resetArm1Encoder() {
        this.pivot1Encoder.setPosition(ArmConstants.ARM_1_INITIAL_ANGLE);
    }

    public void resetArm2Encoder() {
        this.pivot2Encoder.setPosition(ArmConstants.ARM_2_INITIAL_ANGLE);
    }

    public boolean is2D(){
        return this.is2D;
    }
    public void setis2D(boolean is2D){
        this.is2D = is2D;
    }

    /**
     * Changes the intended coordinates by dx, dy, and dz
     */
    public void moveVector(double dx, double dy, double dz) {
        setTargetCoordinates(targetX + dx, targetY + dy, targetZ + dz);
    }

    /**
     * Get the current angles from motor encoders in DEGREES
     *
     * @return [pivot1Angle, pivot2Angle, turretAngle]
     */
    public double[] getCurrentAnglesDeg() {
        double angle1 = pivot1Encoder.getPosition();
        double angle2 = pivot2Encoder.getPosition();
        double angle3 = turretEncoder.getPosition();

        return new double[]{angle1, angle2, angle3};
    }

    public double getTurretAngleDeg() {
        return this.turretEncoder.getPosition();
    }
    /*
     * All the Setter methods below
     */

    public void setPivot1Speed(double speed) {
        this.pivot1.set(speed);
    }

    public void setPivot2Speed(double speed) {
        this.pivot2.set(speed);
    }

    public void setTurretSpeed(double speed) {
        this.turret.set(speed);

    }

    public void setArm1SpeedMultiplier(double mult) {
        this.arm1SpeedMultiplier = mult;
    }

    public void setArm2SpeedMultiplier(double mult) {
        this.arm2SpeedMultiplier = mult;
    }

    public void setManualControlMode(boolean isManual) {
        this.isManual = isManual;
    }

    public void setMaxAndMinOutput1(double speed) {
        this.minOutput = -speed;
        this.maxOutput = speed;
    }

    public void setMaxAndMinOutput2(double speed) {
        this.minOutput2 = -speed;
        this.maxOutput2 = speed;
    }

    public void setTargetAngle1(double angle) {
        this.targetAngle1 = angle;
    }

    public void setTargetAngle2(double angle) {
        this.targetAngle2 = angle;
    }

    public void stopAllMotors() {
        this.pivot1.set(0);
        this.pivot2.set(0);
        this.turret.set(0);
    }

    public boolean getControlMode() {
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

    public boolean getTurretLimitPressed() {
        return !this.turretLimit.get();
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
        // System.out.println("tat - rgggady: " + (MathUtil.roundNearestHundredth(targetAngleTurret + RobotGyro.getGyroAngleDegreesYaw())));

        // if power is NaN, don't run it :D
        if (Double.isNaN(p1Speed) || Double.isNaN(p2Speed)) {
            System.out.println("PID is NaN, so skip");
            return;
        }

        p1Speed = Math.min(maxOutput, Math.max(p1Speed, minOutput));
        p2Speed = Math.min(maxOutput2, Math.max(p2Speed, minOutput2));
        // turretSpeed = Math.min(maxOutput, Math.max(turretSpeed, minOutput));

        setPivot1Speed(p1Speed);
        setPivot2Speed(p2Speed);

        if(!this.is2D){
            double turretSpeed = pidController3.calculate(angles[2], targetAngleTurret);
            if(Double.isNaN(turretSpeed)){
                return;
            }
            setTurretSpeed(turretSpeed);
        }

        //System.out.println("SPEEDS: " + p1Speed + " " + p2Speed + " " + turretSpeed);
    }

    /**
     * sets the coordinate in which the arm "should" move towards
     *
     * @param x       the target x coordinate
     * @param y       the target y coordinate (height)
     * @param z       the target z coordinate
     */
    public void setTargetAngles(double pivot1Angle, double pivot2Angle){
        if(is2D){
            double[] newCoords = ForwardKinematicsUtil.getCoordinatesFromAngles(pivot1Angle, pivot2Angle, 0);
            this.targetAngle1 = pivot1Angle;
            this.targetAngle2 = pivot2Angle;
            this.targetX = newCoords[0];
            this.targetY = newCoords[1];
        }
    }
    public void setTargetCoordinates(double x, double y, double z) {

        if(this.is2D){
            x = Math.max(x, 0);
            z = 0;
        }

        // Updates target Angles
        double[] targetAngles = InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z, getFlipped());

        // Stops any updates if IKU is out of bounds or calculation error occurs
        if (Double.isNaN(targetAngles[0]) || Double.isNaN(targetAngles[1]) || Double.isNaN(targetAngles[2])) {
            System.out.println("Hi this is the Arm Death Prevention Hotline @copyright setIntendedCoordinates");
            return;
        }
        double[] adjustedCoordinates = ForwardKinematicsUtil.getCoordinatesFromAngles(targetAngle1, targetAngle2, targetAngleTurret);

        /*if (!isAtHumanPlayer){
            if (targetAngleTurret > 180) {
                targetAngleTurret -= 360;
            } else if (targetAngleTurret < -180) {
                targetAngleTurret += 360;
            }
        }*/

        //update current coordinates
        updateCurrentCoordinates();

        // Updates target angles
        targetAngle1 = targetAngles[0];
        targetAngle2 = targetAngles[1];
        targetAngleTurret = targetAngles[2];

        // Updates target coordinates
        if(this.is2D) {
            adjustedCoordinates[0] = Math.abs(adjustedCoordinates[0]);
        }
        this.targetX = adjustedCoordinates[0];
        this.targetY = adjustedCoordinates[1];
        this.targetZ = adjustedCoordinates[2];
    }

    public void setIsAtHumanPlayer(boolean atHumanPlayer) {
        this.isAtHumanPlayer = atHumanPlayer;
    }

    public boolean isAtHumanPlayer() {
        return this.isAtHumanPlayer;
    }

    /**
     * Sets the usage of PID control for the arm.
     *
     * @param value True for enabled PID, False for disabled.
     */
    public void setPIDControlState(boolean value) {
        pidOn = value;
        if (!value) {
            stopAllMotors();
        }
    }

    /**
     * Get the usage of PID control for the arm.
     *
     * @return True if enabled, false otherwise
     */
    public boolean getPIDControlOn() {
        return pidOn;
    }

    public boolean getFlipped() {
        return flipped;
    }

    public void setFlipped(boolean flipped) {
        if (!flipped) {
            moveVector(0, 10, 0);
        }
        this.flipped = flipped;
        (new FlipArmCommand(this, flipped)).withInterruptBehavior(InterruptionBehavior.kCancelSelf).schedule();
    }

    public double[] getTargetAngles(){
        return new double[] {targetAngle1, targetAngle2, targetAngleTurret};
    }

    public boolean isAtCoords() {
        double[] curAngles = getCurrentAnglesDeg();
        System.out.println("ARM1 at pos: " + (Math.abs(targetAngle1 - curAngles[0]) < ArmConstants.ANGLE_DELTA));
        System.out.println("ARM2 at pos: " + (Math.abs(targetAngle2 - curAngles[1]) < ArmConstants.ANGLE_DELTA));
        System.out.println("TURR at pos: " + (Math.abs(targetAngleTurret - curAngles[2]) < ArmConstants.ANGLE_DELTA));
        return (Math.abs(targetAngle1 - curAngles[0]) < ArmConstants.ANGLE_DELTA) && (Math.abs(targetAngle2 - curAngles[1]) < ArmConstants.ANGLE_DELTA) && (Math.abs(targetAngleTurret - curAngles[2]) < ArmConstants.ANGLE_DELTA);
    }

    public void correctMotorEncoders(){
        double[] curAngles = getCurrentAnglesDeg();
        if(Math.abs(curAngles[0] - getArm1ConvertedAbsoluteDistance()) > 4){
            pivot1Encoder.setPosition(getArm1ConvertedAbsoluteDistance());
        }
        if(Math.abs(curAngles[1] - getArm2ConvertedAbsoluteDistance()) > 4){
            pivot2Encoder.setPosition(getArm2ConvertedAbsoluteDistance());
        }
    }

    /**
     * Returns the current claw pose. Note that the claw pose is stored as {x, z, y}, where y is the height of the claw. This allows you to use {@link Pose3d#toPose2d()} to get a correct 2D pose.
     *
     * @return A Pose3d object representing the current claw pose.
     */
    public Pose3d getClawPose() {
        this.updateCurrentCoordinates(); // Make sure the coordinates are the latest ones.
        return new Pose3d(MathUtil.inchesToMeters(this.cur_x), MathUtil.inchesToMeters(this.cur_z), MathUtil.inchesToMeters(this.cur_y), new Rotation3d()); // z and y are swapped to handle our global coordinate system (the final coord parameter is the height).
    }

    public double getArm1ConvertedAbsoluteDistance() {
        return arm1AbsoluteEncoder.getAbsolutePosition() * -360 + 335; // getArm1AbsoluteRawDistance();// * 360; // + 2* ArmConstants.ARM_1_INITIAL_ANGLE + 23.5
    }

    public double getArm2ConvertedAbsoluteDistance() {
        return Math.abs(arm2AbsoluteEncoder.getAbsolutePosition()) * -360 + 337; // getArm2AbsoluteRawDistance() * 360 + ArmConstants.ARM_2_INITIAL_ANGLE + 295.69;
    }

    public Command flipTurretCommand(XboxController xboxController) {
        return new GoTowardsCoordinatesCommandTeleop(this, new double[] {ArmConstants.STARTING_COORDS[0] * (isAtHumanPlayer ? -1 : 1), ArmConstants.STARTING_COORDS[1], ArmConstants.STARTING_COORDS[2]}, xboxController, 0.2, 0.2);
    }

    @Override
    public void periodic() {
        // System.out.println("ARM MOTOR ENCODERS: PIV1: " + this.pivot1Encoder.getPosition() + ", PIV2: " + this.pivot2Encoder.getPosition() + ", TURRET: " + this.turretEncoder.getPosition());
        // System.out.println("TARGET COORDS: " + targetX + ", " + targetY + ", " + targetZ);
        // System.out.println("ARM IKU FLIP STATE: " + this.flipped);
        //System.out.println("TARGET ANGLES: " + targetAngle1 + ", " + targetAngle2 + ", " + targetAngleTurret);
        //System.out.println("CURRENT ANGLES " + getCurrentAnglesDeg()[0] + " " + getCurrentAnglesDeg()[1] + " " + getCurrentAnglesDeg()[2]);
        // System.out.println("LIMIT 1: " + getPivot1LimitPressed() + ", LIMIT 2: " + getPivot2LimitPressed() + ", Turret Limit: " + getTurretLimitPressed());
        //System.out.println("CURRENT COORDS: " + Arrays.toString(getCurrentCoordinates()));
        //System.out.println("is at human pakyter: " + this.isAtHumanPlayer);
        // System.out.println("A1: " + getArm1ConvertedAbsoluteDistance() + ", A2: " + getArm2ConvertedAbsoluteDistance() + ", T: " + getTurretConvertedAbsoluteDistance());
        // System.out.println("lim1: " + resetPivot1 + ", lim2: " + resetPivot2);
        // System.out.println("Arm1: " + getArm1ConvertedAbsoluteDistance() + ", Arm2: " + getArm2ConvertedAbsoluteDistance());
        // System.out.println("Abs Enc 1: " + getArm1ConvertedAbsoluteDistance() + ", motor enc 1: " + pivot1Encoder.getPosition());
        // System.out.println("Abs Enc 2: " + getArm2ConvertedAbsoluteDistance() + ", motor enc 2: " + pivot2Encoder.getPosition());
        System.out.println("ABS 1: " + getArm1ConvertedAbsoluteDistance() + ", ABS 2: " + getArm2ConvertedAbsoluteDistance());
        NetworkTablesUtil.getEntry("robot", "target").setDoubleArray(new double[] {targetX, targetY, targetZ});
        NetworkTablesUtil.getEntry("robot", "arm_angles").setDoubleArray(getCurrentAnglesDeg());
        NetworkTablesUtil.getEntry("robot", "arm_p1_ang").setDouble(MathUtil.roundNearestHundredth(getCurrentAnglesDeg()[0]));
        NetworkTablesUtil.getEntry("robot", "arm_p2_ang").setDouble(MathUtil.roundNearestHundredth(getCurrentAnglesDeg()[1]));
        NetworkTablesUtil.getEntry("robot", "arm_tu_ang").setDouble(MathUtil.roundNearestHundredth(getCurrentAnglesDeg()[2]));

        correctMotorEncoders();

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