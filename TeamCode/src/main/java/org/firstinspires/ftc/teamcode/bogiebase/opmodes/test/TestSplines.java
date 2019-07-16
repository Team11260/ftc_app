package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.Path;
import org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.PathPoint;
import org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.Point;
import org.upacreekrobotics.dashboard.Dashboard;

@Autonomous(name = "Test Splines", group = "New")
//@Disabled

public class TestSplines extends AbstractAutonNew {

    Robot robot;

    Path path, path2;

    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {
        robot = new Robot();
        robot.stopTensorFlow();

        //path = new Path(new Point(0, 0), new Point(36, 0), new Point(72, 0), new Point(84, 28), new Point(70, 32), new Point(70, 62), new Point(98, 62), new Point(120, 62));
        //path2 = new Path(new Point(98, 58), new Point(98, 62), new Point(70, 62), new Point(70, 32), new Point(84, 28), new Point(72, 0), new Point(36, 0), new Point(0, 0));
        path = new Path(new Point(0, 0), new Point(60, 0), new Point(15, 30));
        /*telemetry.getSmartdashboard().putGraphPoint("Path", "Lookahead Point", 0, 0);
        telemetry.getSmartdashboard().putGraphPoint("Path", "Closest Point", 0, 0);
        telemetry.getSmartdashboard().putGraph("Path", "Actual", 0, 0);*/

        for(PathPoint point:path.getPoints()) {
            telemetry.getSmartdashboard().putGraph("Path", "Goal", point.getX(), point.getY());
            //telemetry.getSmartdashboard().putGraph("Components", "V", point.getDistance(), point.getVelocity());
            //telemetry.getSmartdashboard().putGraph("Components", "C", point.getDistance(), point.getCurvature());
        }

        /*for(PathPoint point:path.getPoints()) {

            int look = path.getLookAheadPointIndex(new Pose(point.getX(), point.getY(), 0));

            if(look < 0) break;

            path.getCurvatureFromPathPoint(look, new Pose(point.getX(), point.getY(), 0));

            try { Thread.sleep(100); } catch (InterruptedException e) {}
        }*/
    }

    @Override
    public void Run() {
        //robot.setDriveFollowReversed(true);
        robot.driveRunPath(path);
        /*robot.beginIntaking();
        delay(2000);
        robot.finishIntaking();
        robot.setDriveFollowReversed(true);
        robot.driveRunPath(path2);*/
    }

    @Override
    public void Stop() {
        robot.stop();

        Dashboard.startOpMode("Bogie Teleop Two Driver Demo");
    }
}
