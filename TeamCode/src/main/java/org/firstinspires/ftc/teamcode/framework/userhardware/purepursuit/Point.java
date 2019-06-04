package org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit;

public class Point {

    protected final double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Point add(Point point) {
        return new Point(x + point.getX(), y + point.getY());
    }

    public Point subtract(Point point) {
        return new Point(x - point.getX(), y - point.getY());
    }

    public double distance(Point point) {
        return Math.sqrt(Math.pow(point.x - x, 2) + Math.pow(point.y - y, 2));
    }

    public String toString() {
        return "(" + String.format("%.4f", x) + "," + String.format("%.4f", y) + ")";
    }
}
