package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;

public abstract class TankDriveImpl extends TankDrive {
    public static PIDCoefficients DISPLACEMENT_PID = new PIDCoefficients(1, 0, 0.7);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);


    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    public TankDriveImpl() {
        super(Constants.TRACK_WIDTH);

        constraints = new TankConstraints(Constants.BASE_CONSTRAINTS, Constants.TRACK_WIDTH);
        follower = new TankPIDVAFollower(this, DISPLACEMENT_PID, CROSS_TRACK_PID,
                Constants.kV, Constants.kA, Constants.kStatic);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void updateFollower() {
        follower.update(getPoseEstimate());
    }

    public void update() {
        updatePoseEstimate();
        updateFollower();
    }

    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }

    public Pose2d getFollowingError() {
        return follower.getLastError();
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);
}
