package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.TensorFlow;
import org.firstinspires.ftc.teamcode.framework.userhardware.trajectory.SuperArrayList;
import org.firstinspires.ftc.teamcode.framework.userhardware.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.framework.userhardware.trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.framework.userhardware.trajectory.Waypoint;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition.UNKNOWN;

@Autonomous(name = "TensorFlow Test", group = "New")
@Disabled

public class TrajectoryTest extends AbstractAutonNew {

    private Robot robot;

    private static final Waypoint TEST = new Waypoint(new Pose2d(10, 20, 0), PI / 2, -PI / 2);

    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {
        robot = new Robot();
    }

    @Override
    public void Run() {

        while (opModeIsActive()) robot.updateAll();

        //SuperArrayList<Trajectory> trajectories = new TrajectoryBuilder(new Waypoint(new Pose2d(0, 0, -PI / 4), PI / 4)).to(TEST).build();
        //robot.driveFollowTrajectory(trajectories.get(1));
    }

    @Override
    public void Stop() {
        robot.stop();
    }
}
