package org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit;

public abstract class PurePursuitController {

    private final double trackWidth;
    private double lastLeftPosition = 0, lastRightPosition = 0;
    private Pose currentPosition = new Pose();
    private boolean isFollowing = false;
    private Path currentPath = null;

    public PurePursuitController(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public void update() {
        updatePose();

        updateFollower();
    }

    public void updatePose() {

        double leftPosition = getLeftActualPositionInches();
        double rightPosition = getRightActualPositionInches();

        double heading = getActualHeadingDegrees();

        double distance = ((leftPosition - lastLeftPosition) + (rightPosition - lastRightPosition)) / 2;

        currentPosition = new Pose(currentPosition.addVector(new Vector(distance * Math.cos(Math.toRadians(heading)), distance * Math.sin(Math.toRadians(heading)))), heading);

        lastLeftPosition = leftPosition;
        lastRightPosition = rightPosition;
    }

    public void updateFollower() {

        if(!isFollowing || currentPath == null) return;

        int lookahead = currentPath.getLookAheadPointIndex(currentPosition);
        int closest = currentPath.getClosestPointIndex(currentPosition);

        if(lookahead == -1) {
            currentPath = null;
            isFollowing = false;
            return;
        }

        double velocity = currentPath.getPathPointVelocity(closest, currentPosition);
        double curvature = currentPath.getCurvatureFromPathPoint(lookahead, currentPosition);

        double left = velocity * ((2 + curvature * trackWidth)/2);
        double right = velocity * ((2 - curvature * trackWidth)/2);

        setPowers(left, right);
    }

    public void follow(Path path) {
        currentPath = path;
        isFollowing = true;
    }

    public Pose getCurrentPosition() {
        return currentPosition;
    }

    public boolean isFollowing() {
        return isFollowing;
    }

    public void pause() {
        isFollowing = false;
    }

    public void resume() {
        isFollowing = true;
    }

    public abstract double getActualHeadingDegrees();

    public abstract double getLeftActualPositionInches();

    public abstract double getRightActualPositionInches();

    public abstract void setPowers(double l, double r);
}
