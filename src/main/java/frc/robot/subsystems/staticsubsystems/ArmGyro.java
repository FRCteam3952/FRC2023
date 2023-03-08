package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.wpilibj.SerialPort;

public class ArmGyro {

    public static SerialPort arduino;
    public static double gyro_adjust = 0.0;
    public static double saved_angle = 0.0;

    static {
        SerialPort.Port[] port_types = new SerialPort.Port[]{SerialPort.Port.kUSB, SerialPort.Port.kUSB1, SerialPort.Port.kUSB2};
        for (SerialPort.Port port_type : port_types) {
            try {
                arduino = new SerialPort(19200, port_type);
                System.out.println("Arm Gyro Connected");
                break;
            } catch (Exception e) {
                System.out.println("Failed to Connect");
            }
        }
    }

    public static void poke() {
        if (arduino != null && arduino.getBytesReceived() > 0) {
            System.out.println("Arm Gyro Connected");
        } else {
            System.out.println("Arm Gyro not Connected");
        }
    }

    private static String parseReadString(String str) {
        if(!str.contains("S") || !str.contains("E")) {
            return "BAD BAD BAD";
        }
        int firstS = str.indexOf("S", 0);
        int nextD = str.indexOf("E", firstS);

        String s = str.substring(firstS + 1, nextD).replace("\n", "");
        return s;
    }

    public static void update() {
        if (arduino != null && arduino.getBytesReceived() > 0) {
            try {
                String sad = parseReadString(arduino.readString());
                double val = Double.parseDouble(sad);
                saved_angle = val;
            }
            catch (Exception e) {

            }
        }
    }
    public static double getGyroAngle(){
        return saved_angle - gyro_adjust;
    }

    public static void resetAngle() {
        gyro_adjust = saved_angle;
    }
}