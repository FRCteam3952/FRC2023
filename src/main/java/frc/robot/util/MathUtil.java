package frc.robot.util;

public final class MathUtil {
    private MathUtil() {
        throw new UnsupportedOperationException("MathUtil is a utility class and should not be instantiated!");
    }

    public static double distance(double x1, double x2, double y1, double y2) { //2d distance calc
        return Math.sqrt(Math.pow(x1-x2,2) + Math.pow(y1-y2,2));
    }

    public static double distance(double x1, double x2, double y1, double y2, double z1, double z2) { //3d distance calc
        return Math.sqrt(Math.pow(x1-x2,2) + Math.pow(y1-y2,2) + Math.pow(z1-z2,2));
    }

    public static double lawOfCosinesForAngle(double a, double b, double c) { //law of cosines calc
        return Math.toDegrees(Math.acos((Math.pow(a,2) + Math.pow(b,2) - Math.pow(c,2))/(2*(a*b))));
    }

    public static double lawOfCosinesForSide(double a, double b, double includedAngleDeg) {
        return Math.sqrt(a * a + b * b - 2 * a * b * Math.cos(Math.toRadians(includedAngleDeg)));
    }

    public static double lawOfSinesForAngle(double angle, double a, double b) { //law of sines calc
        return Math.toDegrees(Math.asin((b * Math.sin(Math.toRadians(angle))) / a));
    }

    public static double angleBetweenLines(double x1, double y1, double z1, double x2, double y2, double z2){ //angle between lines
        double dotProduct = x1 * x2 + y1 * y2 + z1 * z2;
        return Math.toDegrees(Math.acos(dotProduct/(Math.abs(distance(0,x1,0,y1,0,z1)*distance(0, x2, 0, y2,0,z2)))));
    }   
    public static double squared(double x){
        return x*x;
    } 
}