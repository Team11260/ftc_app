package org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit;

import org.upacreekrobotics.dashboard.Config;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class Path {

    public static double spacing = 0.5, SMOOTHING = 0.995, t = 0.001, k = 0.1;
    public static double MAX_ACCELERATION = 0.1, MIN_SPEED = 0.3, MAX_SPEED = 1.0, LOOK_AHEAD_DISTANCE = 12;

    private int lastPointIndex = 0, lastCurrentPointIndex = 0;

    private double targetAngle = 0;
    private double deltaAngle = 0;
    private double curvature = 0.000001;

    private ArrayList<Point> points;
    private ArrayList<PathPoint> path;

    public Path(Point... points) {
        this.points = new ArrayList<>(Arrays.asList(points));

        build();
    }

    public PathPoint getPoint(int index) {
        return path.get(index);
    }

    public ArrayList<PathPoint> getPoints() {
        return (ArrayList<PathPoint>) path.clone();
    }

    public int getClosestPointIndex(Point currentPosition) {
        double distance = 1000000;
        int index = -1;

        for(int i = lastCurrentPointIndex; i < path.size(); i++) {
            if(currentPosition.distance(path.get(i)) < distance) {
                index = i;
                distance = currentPosition.distance(path.get(i));
            }
        }

        lastCurrentPointIndex = index;

        return index;
    }

    public double getCurvatureFromPathPoint(int index, Pose currentLocation) {
        double x = path.get(index).getX();
        double y = path.get(index).getY();

        double deltaX = x - currentLocation.getX();
        double deltaY = y - currentLocation.getY();

        double angle = atan(Math.abs(deltaX) > 0.3 ? deltaX : 0.3 * Math.signum(deltaX), deltaY);

        targetAngle = angle;

        deltaAngle = currentLocation.getHeading() - angle;

        if(Math.abs(deltaAngle) > 180) deltaAngle = -Math.signum(deltaAngle) * (360 - Math.abs(deltaAngle));

        double c = (Math.abs(deltaAngle) > 90 ? Math.signum(deltaAngle) : Math.sin(Math.toRadians(deltaAngle))) / (path.get(index).distance(currentLocation) / 2);

        if(Double.isInfinite(c) || Double.isNaN(c)) return 0.0;

        curvature = c;

        return c;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getPathPointVelocity(int index, Pose currentLocation) {
        double speed = MAX_SPEED;
        for(int i = index; i < index + 15; i++) {
            speed = Math.min(speed, range(path.get(index).getVelocity() / range(getTrackingError(currentLocation) / 2, 1, 3), MIN_SPEED, MAX_SPEED));
        }
        return speed;
    }

    public int getLookAheadPointIndex(Pose currentPosition) {

        int closest = getClosestPointIndex(currentPosition);

        for(int i = getClosestPointIndex(currentPosition); i < path.size(); i++) {
            curvature = Math.abs(getCurvatureFromPathPoint(i, currentPosition));

            double correction = range(curvature, 1, 5);

            double curvature = 0;

            for(int p = closest; p <= i; p++) {
                curvature += getPointCurvature(closest);
            }

            if(getPointDistance(i) - getPointDistance(closest) > LOOK_AHEAD_DISTANCE) {
                lastPointIndex = i;
                return i;
            }
        }

        if(currentPosition.distance(path.get(path.size() - 1)) > 2) return path.size() - 1;

        return -1;
    }

    public double getTrackingError(Point currentPosition) {
        return path.get(getClosestPointIndex(currentPosition)).distance(currentPosition) + Math.abs(deltaAngle / 8);
    }

    private void build() {

        fill();

        smooth();

        createPath();
    }

    private void fill() {
        ArrayList<Point> newPoints = new ArrayList<>();

        for(int s = 1; s < points.size(); s++) {
            Vector vector = new Vector(points.get(s - 1), points.get(s));

            int numPointsFit = (int) Math.ceil(vector.magnitude() / spacing);

            vector = vector.normalize().scale(spacing);

            for (int i = 0; i < numPointsFit; i++) {
                newPoints.add(points.get(s - 1).addVector(vector.scale(i)));
            }
        }

        newPoints.add(points.get(points.size() - 1));

        points = newPoints;
    }

    private void smooth() {
        ArrayList<Point> newPoints = (ArrayList<Point>) points.clone();

        double change = t;
        while(change >= t) {
            change = 0.0;
            for(int i = 1; i < points.size()-1; i++) {
                Point point = newPoints.get(i);
                newPoints.set(i, newPoints.get(i).addVector(new Vector((1 - SMOOTHING) * (points.get(i).getX() - newPoints.get(i).getX()) + SMOOTHING * (newPoints.get(i - 1).getX() + newPoints.get(i + 1).getX() - (2.0 * newPoints.get(i).getX())), (1 - SMOOTHING) * (points.get(i).getY() - newPoints.get(i).getY()) + SMOOTHING * (newPoints.get(i - 1).getY() + newPoints.get(i + 1).getY() - (2.0 * newPoints.get(i).getY())))));
                change += point.distance(newPoints.get(i));
            }
        }

        points = newPoints;
    }

    private void createPath() {

        path = new ArrayList<>();

        for (int p = 0; p < points.size(); p++) {
            path.add(new PathPoint(points.get(p), getPointDistance(p), getPointCurvature(p), getPointVelocity(p)));
        }

        for (int p = points.size() - 2; p >= 0; p--) {
            path.get(p).setVelocity(getPointNewVelocity(p));
        }

        path.get(path.size() - 1).setVelocity(path.get(path.size() - 2).getVelocity());

        /*for (int p = 0; p < points.size(); p++) {
            System.out.print("(" + p + "," + path.get(p).getVelocity() + "),");
        }*/
    }

    private double getPointDistance(int p) {
        if(p == 0) return 0.0;
        return points.get(p).distance(points.get(p - 1)) + path.get(p - 1).getDistance();
    }

    private double getPointCurvature(int p) {
        if(p <= 0 || p >= points.size() - 1) return 0.0;
        return getCurvature(points.get(p), points.get(p - 1), points.get(p + 1));
    }

    public double getCurvature(Point p1, Point p2, Point p3) {
        double x1 = p1.getX(), y1 = p1.getY();
        double x2 = p2.getX(), y2 = p2.getY();
        double x3 = p3.getX(), y3 = p3.getY();
        if(x1 == x2) x1 += 0.0001;
        double k1 = 0.5 * (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2))/(x1 - x2);
        double k2 = (y1 - y2)/(x1 - x2);
        double b = 0.5 * (Math.pow(x2, 2) - 2 * x2 * k1 + Math.pow(y2, 2) - Math.pow(x3 , 2) + 2 * x3 * k1 - Math.pow(y3, 2))/(x3 * k2 - y3 + y2 - x2 * k2);
        double a = k1 - k2 * b;
        double r = Math.sqrt(Math.pow(x1 - a, 2) + Math.pow(y1 - b, 2));
        double c = 1/r;
        if(Double.isNaN(c)) {
            return 0.0;
        }
        return c;
    }

    private double getPointVelocity(int p) {
        if(p >= points.size() - 1) return 0.3;

        double d = points.get(p).distance(points.get(p + 1));

        if(p <= 0) return Math.max(Math.min(Math.sqrt(2 * MAX_ACCELERATION * d), k / getPointCurvature(p)), 0.2);

        return Math.max(Math.min(Math.sqrt(Math.pow(path.get(p - 1).getVelocity(), 2) + 2 * MAX_ACCELERATION * d), Math.min(k / getPointCurvature(p), MAX_SPEED)), 0.2);
    }

    private double getPointNewVelocity(int p) {
        if(p >= points.size() - 1) return 0.2;

        double d = points.get(p).distance(points.get(p + 1));

        return Math.min(path.get(p).getVelocity(), Math.min(Math.sqrt(Math.pow(path.get(p + 1).getVelocity(), 2) + 2 * MAX_ACCELERATION * d), MAX_SPEED));
    }

    private double atan(double x, double y) {
        if(x < 0 && y > 0) return Math.toDegrees(Math.atan(y/x)) + 180;
        else if(x < 0 && y < 0) return Math.toDegrees(Math.atan(y/x)) - 180;
        return Math.toDegrees(Math.atan(y/x));
    }

    private double range(double num, double low, double high) {
        if(num < low) return low;
        if(num > high) return high;
        return num;
    }
}
