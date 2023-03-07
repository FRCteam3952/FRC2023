package frc.robot.subsystems.staticsubsystems;


import edu.wpi.first.wpilibj.I2C;

/**
 * CODE FROM: <a href="https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/">here</a> and
 * <a href="https://github.com/FRCteam3952/FRC2023/commit/7b3f5fbbcc7946d4214b9712bb282d753be9fc5e">also here</a>
 */
public class MPU6050 {
    private static final int MPU_I2C_ADDR = 0x68;
    private static final int GYRO_REGISTER = 0x43;
    private static final int ACCEL_REGISTER = 0x3B;

    private static final int ERROR_CALC_ITERS = 200;
    private float accX, accY, accZ;
    private float gyroX, gyroY, gyroZ;
    private float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
    private float roll, pitch, yaw;
    private float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
    private float elapsedTime, currentTime, previousTime;

    private int c = 0;

    private final I2C gyro = new I2C(I2C.Port.kOnboard, MPU_I2C_ADDR);

    private final byte[] buffer = new byte[6];

    public MPU6050() {
        try {
            this.setup();
        } catch (InterruptedException e) {
            System.out.println("COULD NOT INITIALIZE MPU6050");
            e.printStackTrace();
        }
    }

    public float getGyroX() {
        return gyroX;
    }

    public float getGyroY() {
        return gyroY;
    }

    public float getGyroZ() {
        return gyroZ;
    }

    private void setup() throws InterruptedException {
        gyro.write(0x6B, 0x00); // In register 6B, place a 0. Why? I have no clue

        gyro.write(0x1C, 0x10); // Talks to the ACCEL_CONFIG register (+- 8g)
        gyro.write(0x1B, 0x10); // Talks to the GYRO_CONFIG register (1000deg/s full scale whatever that means)

        this.calculateIMUError();

        // we don't need this rightttttttt righttttttt it'll be fine right as long as they don't touch the arm
        // Thread.sleep(20); // we pray
    }

    public void periodic() {
        readAccelerometerValues();

        accAngleX = (float) Math.atan(accY / Math.sqrt(accX * accX + accZ * accZ)) * 180f / (float) Math.PI;
        accAngleY = (float) Math.atan(-accX / Math.sqrt(accY * accY + accZ * accZ)) * 180f / (float) Math.PI;

        readGyroValues();

        currentTime = System.nanoTime();
        elapsedTime = (currentTime - previousTime) / 1000000000.0f;

        gyroAngleX = gyroX * elapsedTime;
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

    private void readAccelerometerValues() {
        gyro.read(ACCEL_REGISTER, 6, buffer);

        accX = ((buffer[0] << 8) | buffer[1]) / 16384.0f;
        accY = ((buffer[2] << 8) | buffer[3]) / 16384.0f;
        accZ = ((buffer[4] << 8) | buffer[5]) / 16384.0f;
    }

    private void readGyroValues() {
        gyro.read(GYRO_REGISTER, 6, buffer);

        gyroX = ((buffer[0] << 8) | buffer[1]) / 131.0f; // ??? idk where these came from
        gyroY = ((buffer[2] << 8) | buffer[3]) / 131.0f;
        gyroZ = ((buffer[4] << 8) | buffer[5]) / 131.0f;
    }

    private void calculateIMUError() {
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
