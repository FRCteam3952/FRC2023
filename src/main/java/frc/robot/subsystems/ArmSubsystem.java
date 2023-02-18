package frc.robot.subsystems;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;
import frc.robot.util.ForwardKinematicsUtil;
import frc.robot.util.InverseKinematicsUtil;

// import java.security.DigestInputStream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax pivot1;
    private final CANSparkMax pivot2;
    private final CANSparkMax turret;

    private final RelativeEncoder pivot1Encoder;
    private final RelativeEncoder pivot2Encoder;
    private final RelativeEncoder turretEncoder;

    private final DigitalInput arm1Limit;
    private final DigitalInput arm2Limit;

    private final PIDController pidController1, pidController2;

    private double targetX;
    private double targetY;
    private double targetZ;

    private double cur_x;
    private double cur_y;
    private double cur_z;

    private double targetAngle1;
    private double targetAngle2;
    private double targetAngleTurret;

    private final double kMaxOutput = 0.3;
    private final double kMinOutput = -0.3;

    private boolean pidOn = false;

    
    //arm control constructor
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
        this.pivot1Encoder.setPositionConversionFactor(2.88);
        this.pivot2Encoder.setPositionConversionFactor(6.68);
        this.turretEncoder.setPositionConversionFactor(1);
        this.pivot1Encoder.setPosition(ArmConstants.ARM_1_INITIAL_ANGLE);
        this.pivot2Encoder.setPosition(ArmConstants.ARM_2_INITIAL_ANGLE);
        this.turretEncoder.setPosition(0);

        this.pidController1 = new PIDController(1.69e-2, 0, 0);
        this.pidController1.setTolerance(ArmConstants.ANGLE_DELTA);
        this.pidController2 = new PIDController(9.6e-3, 0, 0);
        this.pidController2.setTolerance(ArmConstants.ANGLE_DELTA);

        // Initialize arm limit switches
        this.arm1Limit = new DigitalInput(PortConstants.PIVOT_1_LIMIT_PORT);
        this.arm2Limit = new DigitalInput(PortConstants.PIVOT_2_LIMIT_PORT);

        // Set starting arm angles
        this.targetAngle1 = ArmConstants.ARM_1_INITIAL_ANGLE;
        this.targetAngle2 = ArmConstants.ARM_2_INITIAL_ANGLE;
        this.targetAngleTurret = 0;

        // Get starting coords from the initial angle constants
        double[] startingCoords = ForwardKinematicsUtil.getCoordinatesFromAngles(ArmConstants.ARM_1_INITIAL_ANGLE,ArmConstants.ARM_2_INITIAL_ANGLE,0);
        this.targetX = startingCoords[0];
        this.targetY = startingCoords[1];
        this.targetZ = startingCoords[2];
        this.cur_x = startingCoords[0];
        this.cur_y = startingCoords[1];
        this.cur_z = startingCoords[2];
    }

    /*
     * Changes the intended coordinates by dx, dy, and dz
     */
    public void moveVector(double dx, double dy, double dz){
        updateCurrentCoordinates();
        setIntendedCoordinates(targetX + dx, targetY + dy, targetZ + dz);
    }

    /*
     * Get the current angles from motor encoders in DEGREES
     */
    public double[] getCurrentAnglesDeg() {
        double angle1 = pivot1Encoder.getPosition(); 
        double angle2 = pivot2Encoder.getPosition();
        double angle3 = turretEncoder.getPosition();
        
        return new double[] {angle1, angle2, angle3};
    }

    /*
     * Get the current angles from motor encoders in radians
     */
    public double[] getCurrentAnglesRad() {
        double angle1 = Math.toRadians(pivot1Encoder.getPosition()); 
        double angle2 = Math.toRadians(pivot2Encoder.getPosition());
        double angle3 = Math.toRadians(turretEncoder.getPosition());
        
        return new double[] {angle1, angle2, angle3};
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
    public void stopAllMotors() {
        this.pivot1.set(0);
        this.pivot2.set(0);
        this.turret.set(0);
    }

    /*
     * Uses motor encoder angles to update the current coordinates
     */
    public void updateCurrentCoordinates(){
        double[] tempAngles = getCurrentAnglesDeg();
        double[] coords = ForwardKinematicsUtil.getCoordinatesFromAngles(tempAngles[0],tempAngles[1],tempAngles[2]);
        this.cur_x = coords[0];
        this.cur_y = coords[1];
        this.cur_z = coords[2];
    }

    /*
     * returns current coordinates
     */
    public double[] getCurrentCoordinates(){
        updateCurrentCoordinates();
        return new double[]{this.cur_x,this.cur_y,this.cur_z};
    }
    
    /*
     * return coordinates in which the arm "should" move towards
     */
    public double[] getIntendedCoordinates(){
        return new double[]{this.targetX,this.targetY,this.targetZ};
    }

    public boolean getPivot1LimitPressed(){
        return !this.arm1Limit.get();
    }
    public boolean getPivot2LimitPressed(){
        return !this.arm2Limit.get();
    }
    public void goTowardIntendedCoordinates(){
        double[] angles = getCurrentAnglesDeg();
        angles[0] = Double.NaN;
        angles[1] = Double.NaN;
        angles[2] = Double.NaN;
        if(angles[0] == Double.NaN || angles[1] == Double.NaN || angles[2] == Double.NaN ||
            targetAngle1 == Double.NaN || targetAngle2 == Double.NaN || targetAngleTurret == Double.NaN) {
            System.out.println("An angle is NaN, so skip");
            return;
        }

        double p1Speed = pidController1.calculate(angles[0], targetAngle1);
        double p2Speed = pidController2.calculate(angles[1], targetAngle2);
        System.out.println(angles[0] + " " + angles[1] + " " );
        // System.out.println(targetAngle1 + " " + targetAngle2 + " " );

        if (Double.isNaN(p1Speed) || Double.isNaN(p2Speed)) {
            System.out.println("PID is NaN, so skip");
            return;
        }

        System.out.println(Math.min(kMaxOutput, Math.max(p1Speed, kMinOutput)) + " " + Math.min(kMaxOutput, Math.max(p2Speed,kMinOutput)));
        // setPivot1Speed(Math.min(kMaxOutput, Math.max(p1Speed, kMinOutput)));
        // setPivot2Speed(Math.min(kMaxOutput, Math.max(p2Speed, kMinOutput)));
    }

    /*
     * sets the coordinate in which the arm "should" move towards
     */
    public void setIntendedCoordinates(double x, double y, double z){
        if(this.targetX == x && this.targetY == y && this.targetZ == z) { // if intended coordinates are same, then don't change target
           return;
        }
        //update intended Angles
        double[] intendedAngles = InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z);

        if(intendedAngles[0] == Double.NaN || intendedAngles[1] == Double.NaN || intendedAngles[2] == Double.NaN) {
            System.out.println("An angle is NaN, so skip");
            return;
        }
        
        targetAngle1 = intendedAngles[0];
        targetAngle2 = intendedAngles[1];
        targetAngleTurret = intendedAngles[2];

        // Updates coordinates
        this.targetX = InverseKinematicsUtil.getCurrentCoordinates()[0];
        this.targetY = InverseKinematicsUtil.getCurrentCoordinates()[1];
        this.targetZ = InverseKinematicsUtil.getCurrentCoordinates()[2];
    }

    public CommandBase calibrateArm() {
        pidOn = false;
        return this.runOnce(
            () -> {
                while (!getPivot2LimitPressed()) {
                    setPivot2Speed(-0.1);
                }
                setPivot2Speed(0);  
                while (!getPivot1LimitPressed()) {
                    setPivot1Speed(-0.1);
                }
                setPivot1Speed(0);
                this.pivot1Encoder.setPosition(ArmConstants.ARM_1_INITIAL_ANGLE);
                this.pivot2Encoder.setPosition(ArmConstants.ARM_2_INITIAL_ANGLE);
                pidOn = true;
            });
    }

    public void setPIDControlOn(boolean value) {
        pidOn = value;
    }

    public boolean getPIDControlOn() {
        return pidOn;
    }

    @Override
    public void periodic() {
        // handles limit switches
        if (getPivot1LimitPressed() && Math.abs(this.pivot1Encoder.getPosition() - ArmConstants.ARM_1_INITIAL_ANGLE) > 0.1) {
            this.pivot1Encoder.setPosition(ArmConstants.ARM_1_INITIAL_ANGLE);

        } 

        if (getPivot2LimitPressed() && Math.abs(this.pivot2Encoder.getPosition() - ArmConstants.ARM_2_INITIAL_ANGLE) > 0.1) {
            this.pivot2Encoder.setPosition(ArmConstants.ARM_2_INITIAL_ANGLE);
        }
        
        //handles PID
        if (pidOn) {
            goTowardIntendedCoordinates();
        }
        
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}