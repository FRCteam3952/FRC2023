package frc.robot.subsystems;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.ForwardKinematicsUtil;
import frc.robot.util.InverseKinematicsUtil;

import java.security.DigestInputStream;

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

    private final PIDController pidController;

    private double x_pos;
    private double y_pos;
    private double z_pos;

    private double cur_x;
    private double cur_y;
    private double cur_z;
    
    //arm control constructor
    public ArmSubsystem() {
        //initialize arm motors
        this.pivot1 = new CANSparkMax(PortConstants.PIVOT1_PORT, MotorType.kBrushless);
        this.pivot2 = new CANSparkMax(PortConstants.PIVOT2_PORT, MotorType.kBrushless);
        this.turret = new CANSparkMax(PortConstants.TURRET_PORT, MotorType.kBrushless);

        //set up arm encoders and position conversion factors
        this.pivot1Encoder = this.pivot1.getEncoder();
        this.pivot2Encoder = this.pivot2.getEncoder();
        this.turretEncoder = this.turret.getEncoder();
        this.pivot1Encoder.setPositionConversionFactor(1/125);
        this.pivot2Encoder.setPositionConversionFactor(1/60);
        this.turretEncoder.setPositionConversionFactor(1/60); //this one i'll have to do math

        //initialize arm limit switches
        this.arm1Limit = new DigitalInput(PortConstants.PIVOT_1_LIMIT_PORT);
        this.arm2Limit = new DigitalInput(PortConstants.PIVOT_2_LIMIT_PORT);

        this.pidController = new PIDController(0.5, 0, 0); // tune later lol
        pidController.setTolerance(ArmConstants.ANGLE_DELTA);

        //get starting coords from the initial angle constants
        double[] startingCoords = ForwardKinematicsUtil.getCoordinatesFromAngles(ArmConstants.ARM_1_INITIAL_ANGLE,ArmConstants.ARM_2_INITIAL_ANGLE,0);
        this.x_pos = startingCoords[0];
        this.y_pos = startingCoords[1];
        this.z_pos = startingCoords[2];
        this.cur_x = startingCoords[0];
        this.cur_y = startingCoords[1];
        this.cur_z = startingCoords[2];
    }

    /*
     * If you're worrying about this ugly method... I don't know either :D
     */
    // Sean no one can understand what tf this is why are there so many random numbers
    public void movePolar(double power, double angle){
        double dist = getDistAway();
        double dP2 = (2 * dist) / Math.sqrt(1065024 - Math.pow((1065.6 - MathUtil.squared(dist)),2)) * power; // Something something derivative of the law of cosines
        double dP1 = -(MathUtil.squared(dist) - 265.64) / (MathUtil.squared(dist) * (51.6 * Math.sqrt(1 - (MathUtil.squared(265.64 + MathUtil.squared(dist)) / (2662.65 * MathUtil.squared(dist)))))) * power;
        //setPivot1Speed(dP1);
        //setPivot2Speed(dP2);
        System.out.println(dP1 + " " + dP2);
    }

    /*
     * Changes the intended coordinates by dx, dy, and dz
     */
    public void moveByElement(double dx, double dy, double dz){
        updateCurrentCoordinates();
        setIntendedCoordinates(cur_x + dx, cur_y + dy, cur_z + dz);
    }

    /*
     * Gets distance away on the vertical plane from 0, 0
     */
    // note from max: i assumed this is the distance from center to the claw on the vertical plane extending along the arm of the claw, and thus we need to take into account the angle of the turret; if this assumption is incorrect, revert my code and comment
    public double getDistAway(){
        updateCurrentCoordinates();
        return MathUtil.distance((Math.sin(getCurrentAnglesRad()[2]) * cur_x + Math.cos(getCurrentAnglesRad()[2]) * cur_z), 0, cur_y, 0);
    }

    /*
     * Get the current angles from motor encoders in degrees
     */
    public double[] getCurrentAnglesDeg() {
        double angle1 = pivot1Encoder.getPosition(); // "degrees" - sean
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
        return new double[]{this.x_pos,this.y_pos,this.z_pos};
    }

    /*
     * sets the coordinate in which the arm "should" move towards
     */
    public void setIntendedCoordinates(double x, double y, double z){
        if(this.x_pos == x && this.y_pos == y && this.z_pos == z) { // if intended coordinates are same, then don't change target
           return;
        }
        var ikuAngles = InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z);
        var currentAngles = getCurrentAnglesDeg();

        double p1Speed = pidController.calculate(currentAngles[0], ikuAngles[0]);
        double p2Speed = pidController.calculate(currentAngles[1], ikuAngles[1]);
        double turretSpeed = pidController.calculate(currentAngles[2], ikuAngles[2]);

        System.out.println(p1Speed + " " + p2Speed + " " + turretSpeed);
        /*setPivot1Spdeed(p1Speed);
        setPivot2Speed(p2Speed);
        setTurretSpeed(turretSpeed);*/

        // Updates coordinates
        this.x_pos = x;
        this.y_pos = y;
        this.z_pos = z;
    }

    @Override
    public void periodic() {
        // handles limit switches
        if (this.arm1Limit.get()) {
           this.pivot1Encoder.setPosition(ArmConstants.ARM_1_INITIAL_ANGLE);
        }
        if (this.arm2Limit.get()) {
            this.pivot2Encoder.setPosition(ArmConstants.ARM_2_INITIAL_ANGLE);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}