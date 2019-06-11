package org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit;

import java.util.ArrayList;
import java.util.Arrays;

public class Path {

    private ArrayList<Point> points;

    public Path(Point... points) {
        this.points = new ArrayList<>(Arrays.asList(points));
    }

    public void getLookaheadPoint(Pose currentPosition) {
        //getIntersectionPoints(currentPosition, 12, )
    }

    public void getClosestPoint(Pose currentPosition) {

    }

    private ArrayList<Point> getIntersectionPoints(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {

        if(Math.abs(linePoint1.getX() - linePoint2.getX()) < 0.0001) linePoint1 = new Point(Math.signum(linePoint1.getX() - linePoint2.getX()) + 0.0001, linePoint1.getY());

        if(Math.abs(linePoint1.getY() - linePoint2.getY()) < 0.0001) linePoint1 = new Point(linePoint1.getX(), Math.signum(linePoint1.getY() - linePoint2.getY()) + 0.0001);

        double m = (linePoint2.getY() - linePoint1.getY()) / (linePoint2.getX() - linePoint1.getX());

        Point transformedLinePoint1 = linePoint1.subtract(circleCenter);

        double quadraticA = 1.0 + Math.pow(m, 2);

        double quadraticB = (2.0 * m * transformedLinePoint1.getY()) - (2.0 * Math.pow(m, 2) * transformedLinePoint1.getX());

        double quadraticC = ((Math.pow(m, 2) * Math.pow(transformedLinePoint1.getX(), 2)) - (2.0 * m * transformedLinePoint1.getX() * transformedLinePoint1.getY()) + Math.pow(transformedLinePoint1.getY(), 2) - Math.pow(radius, 2));

        ArrayList<Point> intersectionPoints = new ArrayList<>();

        double rootX = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        double rootY = m * (rootX - transformedLinePoint1.getX()) + transformedLinePoint1.getY();

        rootX += circleCenter.getX();
        rootY += circleCenter.getY();

        if((Math.min(linePoint1.getX(), linePoint2.getX()) <= rootX && rootX <= Math.max(linePoint1.getX(), linePoint2.getX())) || (Math.min(linePoint1.getY(), linePoint2.getY()) <= rootY && rootY <= Math.max(linePoint1.getY(), linePoint2.getY()))) {
            intersectionPoints.add(new Point(rootX, rootY));
        }

        rootX = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        rootY = m * (rootX - transformedLinePoint1.getX()) + transformedLinePoint1.getY();

        rootX += circleCenter.getX();
        rootY += circleCenter.getY();

        if((Math.min(linePoint1.getX(), linePoint2.getX()) <= rootX && rootX <= Math.max(linePoint1.getX(), linePoint2.getX())) || (Math.min(linePoint1.getY(), linePoint2.getY()) <= rootY && rootY <= Math.max(linePoint1.getY(), linePoint2.getY()))) {
            intersectionPoints.add(new Point(rootX, rootY));
        }

        return intersectionPoints;
    }
}
