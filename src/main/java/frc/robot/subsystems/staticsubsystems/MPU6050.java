package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.wpilibj.I2C;

public class MPU6050{
    private static final int MPU_I2C_ADDR = 0x68;
    private static final int GYRO_REGISTER = 0x43;
    private static final int ACCEL_REGISTER = 0x3B;

    private static final int ERROR_CALC_ITERS = 200;
    private static float accX, accY, accZ;
    private static float gyroX, gyroY, gyroZ;
    private static float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
    private static float roll, pitch, yaw;
    private static float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
    private static float elapsedTime, currentTime, previousTime;

    public static double gyro_adjust_roll = 0.0;
    public static double gyro_adjust_pitch = 0.0;

    private static int c = 0;

    private static final I2C gyro = new I2C(I2C.Port.kOnboard, MPU_I2C_ADDR);

    private static final byte[] buffer = new byte[6];

    static{
        try {
            setup();
        } catch (InterruptedException e) {
            System.out.println("COULD NOT INITIALIZE MPU6050");
            e.printStackTrace();
        }
    }
    public static void poke(){
        System.out.println("Arm Gyro Initialized");
    }

    public static double getRoll(){
        return roll - gyro_adjust_roll;
    }

    public static double getPitch() {
        return pitch - gyro_adjust_pitch;
    }

    public static double getYaw() {
        return yaw;
    }

    public static void resetAngle() {
        gyro_adjust_roll = roll;
    }
    public static void resetAngleYaw(){
        gyro_adjust_pitch = yaw;
    }

    private static void setup() throws InterruptedException {
        gyro.write(0x6B, 0x00); // Resets Gyro 
        gyro.write(0x1C, 0x10); // Talks to the ACCEL_CONFIG register (+- 8g)
        gyro.write(0x1B, 0x10); // Talks to the GYRO_CONFIG register (1000deg/s full scale whatever that means)

        Thread.sleep(10);
        calculateIMUError();
        Thread.sleep(10);
    }

    public static void update() {
        readAccelerometerValues();

        accAngleX = (float) Math.atan(accY / Math.sqrt(accX * accX + accZ * accZ)) * 180f / (float) Math.PI; 
        accAngleY = (float) Math.atan(-accX / Math.sqrt(accY * accY + accZ * accZ)) * 180f / (float) Math.PI;
 
        accAngleX -= accErrorX;
        accAngleY -= accErrorY;

        readGyroValues();
 
        currentTime = System.nanoTime();
        elapsedTime = (currentTime - previousTime) / 1000000000.0f;

        gyroAngleX = gyroX * elapsedTime;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             gyroAngleX = gyroX * elapsedTime;
        gyroAngleY = gyroY * elapsedTime;
        gyroAngleZ = gyroZ * elapsedTime;

        gyroX -= gyroErrorX;
        gyroY -= gyroErrorY;
        gyroZ -= gyroErrorZ;

        // "Complementary filter"
        roll = 0.98f * (roll + gyroAngleX) + 0.02f * accAngleX;
        pitch = 0.98f * (pitch + gyroAngleY) + 0.02f * accAngleY;
        yaw = gyroAngleZ;

        previousTime = currentTime;
    }

    private static void readAccelerometerValues() {
        gyro.read(ACCEL_REGISTER, 6, buffer);

        accX = ((buffer[0] << 8) | buffer[1]) / 16384.0f;
        accY = ((buffer[2] << 8) | buffer[3]) / 16384.0f;
        accZ = ((buffer[4] << 8) | buffer[5]) / 16384.0f;

    }

    private static void readGyroValues() {
        gyro.read(GYRO_REGISTER, 6, buffer);

        gyroX = ((buffer[0] << 8) | buffer[1]) / 131.0f; // ??? idk where these came from
        gyroY = ((buffer[2] << 8) | buffer[3]) / 131.0f;
        gyroZ = ((buffer[4] << 8) | buffer[5]) / 131.0f;
;    }


    private static void calculateIMUError() {
        while(c < ERROR_CALC_ITERS) {
            readAccelerometerValues();

            // sum readings
            accErrorX += ((Math.atan((accY) / Math.sqrt(accX * accX + accZ * accZ))) * 180f / Math.PI);
            accErrorY += ((Math.atan((-accX) / Math.sqrt(accY * accY + accZ * accZ))) * 180f / Math.PI);

            c++; // hheh c++
        }

        accErrorX /= ERROR_CALC_ITERS;
        accErrorY /= ERROR_CALC_ITERS;
        c = 0;

        while(c < ERROR_CALC_ITERS) {
            readGyroValues();

            // sum readings
            gyroErrorX += gyroX;
            gyroErrorY += gyroY;
            gyroErrorZ += gyroZ;

            c++; // hheh c++
        }

        gyroErrorX /= ERROR_CALC_ITERS;
        gyroErrorY /= ERROR_CALC_ITERS;
        gyroErrorZ /= ERROR_CALC_ITERS;
    }
}