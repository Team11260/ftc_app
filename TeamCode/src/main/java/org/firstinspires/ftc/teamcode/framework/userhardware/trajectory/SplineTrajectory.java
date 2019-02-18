package org.firstinspires.ftc.teamcode.framework.userhardware.trajectory;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;

public class SplineTrajectory extends Trajectory {

    private Path path;
    private MotionProfile axialProfile;

    private PIDController axialController, lateralController, headingController;

    private double duration;
    private boolean complete = false;
    private double error = 0, axialError = 0, lateralError = 0, averageHeadingError = 0;

    public SplineTrajectory(Path path) {
        this(path, 0, false);
    }

    public SplineTrajectory(Path path, double startAccelerate, boolean stopAccelerate) {

        this.path = path;
        this.axialProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, startAccelerate, 0),
                new MotionState(path.length(), 0, stopAccelerate ? -30 : 0, 0),
                30,
                30,
                30
        );
        duration = axialProfile.duration();

        this.axialController = new PIDController(AXIAL_P, AXIAL_I, AXIAL_D);
        this.lateralController = new PIDController(LATERAL_P, LATERAL_I, LATERAL_D);
        this.headingController = new PIDController(HEADING_P, HEADING_I, HEADING_D);

    }

    long lastupdate = 0;

    @Override
    public synchronized Pose2d update(double t, Pose2d pose) {
        if (t >= duration) complete = true;
        Pose2d targetPose = path.get(axialProfile.get(t).getX());
        double theta = Angle.norm(path.deriv(axialProfile.get(t).getX()).pos().angle());

        Pose2d targetVelocity = path.deriv(axialProfile.get(t).getX()).times(axialProfile.get(t).getV());
        Vector2d trackingError = pose.pos().minus(targetPose.pos()).rotated(-theta);

        Vector2d trackingCorrection = new Vector2d(
                axialController.update(trackingError.getX()),
                lateralController.update(trackingError.getY())
        );
        trackingCorrection = trackingCorrection.rotated(theta);

        double headingError = Angle.norm(pose.getHeading() - targetPose.getHeading());
        double headingCorrection = headingController.update(headingError);

        Pose2d correction = new Pose2d(trackingCorrection, headingCorrection);

        if (lastupdate == 0) lastupdate = System.currentTimeMillis();
        else {
            long now = System.currentTimeMillis();
            long dt = now - lastupdate;
            lastupdate = now;
            error += trackingError.norm() * dt;
            axialError += trackingError.getX() * dt;
            lateralError += trackingError.getY() * dt;
            averageHeadingError += headingError * dt;
        }

        return targetVelocity.minus(correction);
    }

    @Override
    public synchronized boolean isComplete() {
        return complete;
    }

//    public synchronized double averageError() {
//        return complete ? error / duration: 0;
//    }
//
//    public synchronized double avergeLateralError() {return complete ? lateralError / duration: 0;}
//
//    public synchronized double averageAxialError() {return complete ? axialError / duration: 0;}
//
//    public synchronized double averageHeadingError() {return complete ? averageHeadingError / duration: 0;}

    @Override
    public Pose2d getPose(double t) {
        return path.get(axialProfile.get(t).getX());
    }

    @Override
    public double getV(double t) {
        return axialProfile.get(t).getV();
    }

    @Override
    public double duration() {
        return axialProfile.duration();
    }

}
