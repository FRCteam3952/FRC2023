package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * I2C Gyro, copied from: <a href="https://www.reddit.com/r/FRC/comments/2u4bvf/help_with_gyro">...</a>
 */
public class ArmGyro extends SubsystemBase {
    private static final byte MPU6050_ADDRESS = 0x68;
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
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {}
}