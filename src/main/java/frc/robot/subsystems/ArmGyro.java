package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/**
 * I2C Gyro, copied from: <a href="https://www.reddit.com/r/FRC/comments/2u4bvf/help_with_gyro">...</a>
 */
public class ArmGyro extends SubsystemBase {
    /*private static final byte MPU6050_ADDRESS = 0x68;
    private static final int REGISTER_PWR_MGMT_1 = 0x68;
    private static final int REGISTER_GYRO = 0x43;

    private static final I2C accelerometer = new I2C(I2C.Port.kMXP, MPU6050_ADDRESS);
    private static final byte[] buffer = new byte[6];

    static {
        accelerometer.write(REGISTER_PWR_MGMT_1, 0);
    }

    public static void poke() {
        System.out.println("I2C Gyro init");
    }

    public static int[] getGyroMeasurements() {
        accelerometer.read(REGISTER_GYRO, 6, buffer);

        int x = (buffer[0] << 8) | buffer[1];
        int y = (buffer[2] << 8) | buffer[3];
        int z = (buffer[4] << 8) | buffer[5];

        return new int[] {x, y, z};
    }*/

    public static SerialPort arduino;
    public static double gyro_adjust = 0.0;

    static {
        try{
            arduino = new SerialPort(19200, SerialPort.Port.kUSB);
            System.out.println("Arm Gyro Connected");
        } catch (Exception e){
            System.out.println("Failed to Connect, trying kUSB1");
            try{
                arduino = new SerialPort(19200, SerialPort.Port.kUSB1);
                System.out.println("Arm Gyro Connected");
            } catch (Exception e1){
                System.out.println("Failed to Connect, trying kUSB2");
                try{
                    arduino = new SerialPort(19200, SerialPort.Port.kUSB2);
                    System.out.println("Arm Gyro Connected");
                } catch (Exception e2){
                    System.out.println("Failed to Connect, all connections failed");
                }
            }
        }
    }

    public static void poke(){
        if(arduino.getBytesReceived() > 0){
            System.out.println("Arm Gyro Connected");
        }
        else{
            System.out.println("Arm Gyro not Connected");
        }
    }
    public static double getGyroAngle() {
        return Double.parseDouble(arduino.readString()) + gyro_adjust;
    }

    public static void setGyroAngle(double angle){
        gyro_adjust = -getGyroAngle() + angle;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {}
}