package frc.robot.util;

class Point {
    public double x;
    public double y;
    public double z;

    public Point (double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Point add(Point p) {
        return new Point(this.x + p.x, this.y + p.y, this.z + p.z);
    }

    public double[] as_list() {
        return new double[] {this.x, this.y, this.z};
    }
}