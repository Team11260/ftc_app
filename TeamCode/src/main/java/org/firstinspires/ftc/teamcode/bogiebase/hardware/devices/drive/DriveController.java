package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.drive;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userhardware.PIDController;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.DriveSegment;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.Path;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.Segment;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.SplineSegment;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.TurnSegment;
import org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.Point;
import org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.Pose;
import org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.Vector;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

import java.text.DecimalFormat;
import java.util.ArrayList;

import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.DRIVE_COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.DRIVE_DUMP_TEAM_MARKER_DELAY;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.DRIVE_MINERAL_LIFT_RAISED_SCALAR;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.DRIVE_RELEASE_WHEELS_POWER;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.DRIVE_RELEASE_WHEEL_DELAY;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.DRIVE_TEAM_MARKER_EXTENDED;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.DRIVE_TEAM_MARKER_RETRACTED;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.DRIVE_TEAM_MARKER_TELEOP_RETRACTED;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.MatchState;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.MineralLiftState;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.currentMatchState;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.currentMineralLiftState;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.currentPath;
import static org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry.LogMode.INFO;

public class DriveController extends SubsystemController {

    private Drive drive;

    private PIDController anglePID, straightPID, distancePID;

    private double baseHeading = 0;

    private double turnY = 0, turn_z = 0, leftPower = 0, rightPower = 0, Drive_Power = 1.0;

    private ElapsedTime runtime;

    private DecimalFormat DF;

    //Utility Methods
    public DriveController() {
        init();
    }

    public synchronized void init() {

        opModeSetup();

        runtime = new ElapsedTime();

        DF = new DecimalFormat("#.###");
        //Put general setup here
        drive = new Drive(hardwareMap);
        anglePID = new PIDController(15, 0.1, 250, 0.3, 0.08);//D was 150
        //anglePID.setLogging(true);
        straightPID = new PIDController(50, 0.5, 50, 1, 0);
        distancePID = new PIDController(0.6, 0.1, 0, 2, 0.1);
    }

    public synchronized void update() {
        telemetry.addData(DoubleTelemetry.LogMode.TRACE, "Left drive power: " + drive.getLeftPower());
        telemetry.addData(DoubleTelemetry.LogMode.TRACE, "Right drive power: " + drive.getRightPower());
        telemetry.addData(DoubleTelemetry.LogMode.TRACE, "Left drive position: " + drive.getLeftPosition());
        telemetry.addData(DoubleTelemetry.LogMode.TRACE, "Right drive position: " + drive.getRightPosition());
    }

    public synchronized void stop() {
        drive.stop();
    }

    //Autonomous Methods
    public synchronized void runDrivePath(Path path) {

        boolean lastPathPaused = false;

        if (currentPath != null && currentPath.isPaused()) {
            lastPathPaused = true;
        }

        currentPath = path;
        currentPath.reset();

        if (lastPathPaused) currentPath.pause();

        telemetry.addData(INFO, "Starting path: " + currentPath.getName() + "  paused: " + currentPath.isPaused() + "  done: " + currentPath.isDone());

        while (!path.isDone() && opModeIsActive()) {

            //Path is done
            if (path.getNextSegment() == null) break;

            telemetry.addData(INFO, "Starting segment: " + path.getCurrentSegment().getName() + " in path: " + currentPath.getName() + "  paused: " + currentPath.isPaused() + "  done: " + currentPath.isDone());

            if (path.getCurrentSegment().getType() == Segment.SegmentType.TURN) {
                turnToSegment((TurnSegment) path.getCurrentSegment());
            } else if (path.getCurrentSegment().getType() == Segment.SegmentType.DRIVE) {
                driveToSegment((DriveSegment) path.getCurrentSegment());
            } else if (path.getCurrentSegment().getType() == Segment.SegmentType.SPLINE) {
                splineToSegment((SplineSegment) path.getCurrentSegment());
            }

            telemetry.addData(INFO, "Finished segment: " + path.getCurrentSegment().getName() + " in path: " + currentPath.getName() + "  paused: " + currentPath.isPaused() + "  done: " + currentPath.isDone());
        }

        telemetry.addData(INFO, "Finished path: " + currentPath.getName() + "  paused: " + currentPath.isPaused() + "  done: " + currentPath.isDone());
    }

    public synchronized void turnToSegment(TurnSegment segment) {

        double angle = segment.getAngle(), speed = segment.getSpeed(), error = segment.getError(), period = segment.getPeriod();

        telemetry.addData(INFO, "___________________");
        telemetry.addData(INFO, "");
        telemetry.addData(INFO, "");
        telemetry.addData(INFO, "Angle = " + segment.getAngle() + " Speed = " + segment.getSpeed()
                + " error = " + segment.getError() + " Period = " + segment.getPeriod());
        if (angle > 180) {
            baseHeading = angle - 360;
        } else if (angle < -180) {
            baseHeading = angle + 360;
        } else {
            baseHeading = angle;
        }

        telemetry.addData(INFO, "Angle: " + angle);
        telemetry.addData(INFO, "Baseheading: " + baseHeading);

        anglePID.reset();
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double power;
        while (opModeIsActive()) {

            double currentHeading;
            int angleCall = 0;
            int loop = 0;
            runtime.reset();

            //While we are not in the error band keep turning
            while (!atPosition(baseHeading, currentHeading = getHeading(), error) && (baseHeading + error > 180 ? !atPosition((((baseHeading + error) - 180) - 180) - error, currentHeading, error) : true) && (baseHeading - error < -180 ? !atPosition((((baseHeading - error) + 180) + 180) + error, currentHeading, error) : true) && opModeIsActive()) {

                if (segment.isDone()) {
                    setPower(0, 0);
                    return;
                }
                if (!segment.isRunning()) {
                    setPower(0, 0);
                    continue;
                }

                //Use the PIDController class to calculate power values for the wheels
                if (angle > 180) {
                    angleCall = 1;
                    power = anglePID.output(angle, currentHeading < 0 && currentHeading < baseHeading + 30 ? 360 + currentHeading : currentHeading);
                } else if (angle < -180) {
                    angleCall = 2;
                    power = anglePID.output(angle, currentHeading > 0 && currentHeading > baseHeading - 30 ? currentHeading - 360 : currentHeading);
                } else if (angle - currentHeading > 180) {
                    angleCall = 3;
                    power = anglePID.output(angle, 360 + currentHeading);
                } else if (currentHeading - angle > 180) {
                    angleCall = 4;
                    power = anglePID.output(angle, angle - (360 - (currentHeading - angle)));
                } else {
                    angleCall = 5;
                    power = anglePID.output(angle, currentHeading);
                }

                if (power > speed) power = speed;
                if (power < -speed) power = -speed;

                /*telemetry.addData(INFO,
                        "                                                                                                                                 call = "
                                + angleCall + " Power = " + DF.format(power) + " Angle = " + getHeading());*/
                setPower(-power, power);

                loop++;
            }

            runtime.reset();

            while (runtime.milliseconds() < period) {
                if ((abs(getHeading() - baseHeading)) > error && (abs(getHeading() + baseHeading)) > error) break;
            }
            if ((abs(getHeading() - baseHeading)) > error && (abs(getHeading() + baseHeading)) > error) continue;


            telemetry.addData(INFO, "");
            telemetry.addData(INFO, "Average loop time for turn: " + DF.format(runtime.milliseconds() / loop)
                    + " Time = " + runtime.milliseconds() + " Loops = " + loop);
            telemetry.addData(INFO, "Left encoder position: " + DF.format(drive.getLeftPosition()) + "  Right encoder position: " + DF.format(drive.getRightPosition()));
            telemetry.addData(INFO, "Final angle: " + DF.format(getHeading()));
            telemetry.update();

            drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return;
        }
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public synchronized void driveToSegment(DriveSegment segment) {
        telemetry.addData(INFO, "Drive Segment is starting");
        telemetry.addData(INFO, "");
        telemetry.addData(INFO, "");
        telemetry.addData(INFO, "");

        //AbstractOpMode.delay(100);

        double distance = segment.getDistance(), speed = segment.getSpeed(), angle = baseHeading;
        if (segment.getAngle() != null) angle = segment.getAngle();
        int error = segment.getError();

        baseHeading = angle;

        straightPID.reset(); //Resets the PID values in the PID class to make sure we do not have any left over values from the last segment
        distancePID.reset();
        straightPID.setMinimumOutput(0);
        int position = (int) (distance * DRIVE_COUNTS_PER_INCH); //
        double turn;
        speed = range(speed);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPositionP(5);
        telemetry.update();
        double leftPower, rightPower;
        double power;

        double currentHeading;

        int loop = 0;
        runtime.reset();

        while ((!atPosition(position, drive.getLeftPosition(), error) && !atPosition(position, drive.getRightPosition(), error)) && opModeIsActive()) {

            if (segment.isDone()) {
                setPower(0, 0);
                return;
            }
            if (!segment.isRunning()) {
                setPower(0, 0);
                continue;
            }

            currentHeading = getHeading();

            power = range(distancePID.output(position, (drive.getRightPosition() + drive.getLeftPosition()) / 2.0));

            if (angle - currentHeading > 180) {
                turn = anglePID.output(angle, 360 + currentHeading);
            } else if (currentHeading - angle > 180) {
                turn = anglePID.output(angle, angle - (360 - (currentHeading - angle)));
            } else {
                turn = anglePID.output(angle, currentHeading);
            }

            if (power > 0) {
                leftPower = range(power * (speed - turn));
                rightPower = range(power * (speed + turn));
            } else {
                leftPower = range(power * (speed + turn));
                rightPower = range(power * (speed - turn));
            }

            drive.setPower(leftPower, rightPower);

            loop++;
        }

        telemetry.addData(INFO, "Average loop time for drive: " + runtime.milliseconds() / loop);
        telemetry.addData(INFO, "Left encoder position: " + drive.getLeftPosition() + "  Right encoder position: " + drive.getRightPosition());
        telemetry.addData(INFO, "Final angle: " + getHeading());
        telemetry.update();

        drive.setPower(0, 0);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public synchronized void splineToSegment(SplineSegment segment) {

        if(segment.getStartingPosition() != null) drive.setPoseEstimate(segment.getStartingPosition());

        drive.followTrajectory(drive.trajectoryBuilder().splineTo(segment.getTrajectory()).build());

        while (drive.isFollowingTrajectory() && opModeIsActive()) {
            drive.update();
            telemetry.addData(INFO, "Pose: ", drive.getPoseEstimate());
            telemetry.update();
        }

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public synchronized void runPath(org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.Path path) {

        //drive.setSpeedPIDF(new PIDFCoefficients(10, 0, 0, drive.getSpeedPIDF().f));

        ArrayList<Point> points = new ArrayList<>();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose currentPosition = new Pose();

        double l = 0, r = 0;

        while (opModeIsActive()) {
            double heading = getHeading();

            double ll = drive.getLeftPosition();
            double rr = drive.getRightPosition();
            double d = (((ll - l) + (rr - r)) / 2) / DRIVE_COUNTS_PER_INCH;
            l = ll;
            r = rr;
            currentPosition = new Pose(currentPosition.addVector(new Vector(d * Math.cos(Math.toRadians(heading)), d * Math.sin(Math.toRadians(heading)))), heading);

            points.add(currentPosition);

            int lookahead = path.getLookAheadPointIndex(currentPosition);
            int closest = path.getClosestPointIndex(currentPosition);

            if(lookahead == -1) break;

            //telemetry.getSmartdashboard().putGraph("Path", "Actual", currentPosition.getX(), currentPosition.getY());
            //telemetry.getSmartdashboard().putGraphPoint("Path", "Lookahead Point", path.getPoints().get(lookahead).getX(), path.getPoints().get(lookahead).getY());
            //telemetry.getSmartdashboard().putGraphPoint("Path", "Closest Point", path.getPoints().get(closest).getX(), path.getPoints().get(closest).getY());

            //double v = path.getPathPointVelocity(closest) / (path.getTrackingError(currentPosition) > 2 ? path.getTrackingError(currentPosition) : 2);
            double v = Math.min(path.getPathPointVelocity(closest, currentPosition), path.getPathPointVelocity(lookahead, currentPosition));
            double c = path.getCurvatureFromPathPoint(lookahead, currentPosition);

            double left = v * ((2 + c * TRACK_WIDTH)/2);
            double right = v * ((2 - c * TRACK_WIDTH)/2);

            drive.setPower(left, right);

            //telemetry.addDataDB(INFO, "Left: " + (float)left + "  Right: " + (float)right + "  Left C: " + (float)drive.getLeftMotorCurrentDraw() + "  Right C: " + (float)drive.getRightMotorCurrentDraw() + " X: " + (float)currentPosition.getX() + "  Y: " + (float)currentPosition.getY() + "  Heading: " + (float)currentPosition.getHeading() + "  Target Heading: " + (float)path.getTargetAngle() + "  C: " + (float)c + "  I: " + lookahead + "  Point: " + path.getPoint(lookahead));
            //telemetry.update();
        }

        for(Point point:points) telemetry.getSmartdashboard().putGraph("Path", "Actual", point.getX(), point.getY());

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public synchronized void runTrajectory(Trajectory trajectory) {

        drive.followTrajectory(trajectory);

        telemetry.addData(INFO, "Running trajectory: " + trajectory.getSegments().toString() + " " + drive.isFollowingTrajectory() + " " + opModeIsActive());
        telemetry.update();

        while (drive.isFollowingTrajectory() && opModeIsActive()) {
            drive.update();
            telemetry.addData(INFO, "Pose", drive.getPoseEstimate());
            telemetry.update();
        }

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public synchronized TrajectoryBuilder getTrajectoryBuilder() {
        return drive.trajectoryBuilder();
    }

    public synchronized void setPosition(int position, double power) {
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.setPower(range(power), range(power));
        drive.setTargetPosition(position);
    }

    public synchronized void autonReleaseWheelsSequence() {
        setPower(DRIVE_RELEASE_WHEELS_POWER, DRIVE_RELEASE_WHEELS_POWER);
        delay(DRIVE_RELEASE_WHEEL_DELAY);
        setPower(0, 0);
    }

    public void autonDriveToWallSequence() {
        while (RobotState.currentPath.getCurrentSegment().getName().equals("drive to wall") && opModeIsActive() && drive.getLeftMotorCurrentDraw() < 10000 && drive.getRightMotorCurrentDraw() < 10000) {
            telemetry.addDataPhone(INFO, "LEFT:" + drive.getLeftMotorCurrentDraw());
            telemetry.addDataPhone(INFO, "RIGHT: " + drive.getRightMotorCurrentDraw());
        }

        if (!RobotState.currentPath.getCurrentSegment().getName().equals("drive to wall")) return;

        RobotState.currentPath.nextSegment();
    }

    public double getHeading() {
        return drive.getHeading();
    }

    public double getPitch() {
        return drive.getPitch();
    }

    public int getLeftPosition() {
        return drive.getLeftPosition();
    }

    public int getRightPosition() {
        return drive.getRightPosition();
    }

    public synchronized void resetAngleToZero() {
        drive.resetAngleToZero();
    }

    //TeleOp Methods
    public synchronized void setPowerNoEncoder(double left, double right) {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPower(range(left), range(right));
    }

    public synchronized void setPower(double left, double right) {

        if ((currentMineralLiftState == MineralLiftState.IN_MOTION ||
                currentMineralLiftState == MineralLiftState.DUMP_POSITION) &&
                currentMatchState == MatchState.TELEOP) {
            left *= DRIVE_MINERAL_LIFT_RAISED_SCALAR;
            right *= DRIVE_MINERAL_LIFT_RAISED_SCALAR;
        }

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setPower(range(left), range(right));
    }

    public synchronized void setY(double y) {
        turnY = (float) scaleInput(y);
    }

    public synchronized void setZ(double z) {
        turn_z = z;
        turn_z = (float) scaleInput(turn_z);
    }

    public synchronized void updateYZDrive() {
        if ((currentMineralLiftState == MineralLiftState.IN_MOTION ||
                currentMineralLiftState == MineralLiftState.DUMP_POSITION) &&
                currentMatchState == MatchState.TELEOP) {
            leftPower = range((turnY + turn_z) * (Drive_Power * DRIVE_MINERAL_LIFT_RAISED_SCALAR));
            rightPower = range((turnY - turn_z) * (Drive_Power * DRIVE_MINERAL_LIFT_RAISED_SCALAR));
        } else {
            leftPower = range((turnY + turn_z) * Drive_Power);
            rightPower = range((turnY - turn_z) * Drive_Power);
        }

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPower(leftPower, rightPower);
    }

    //Util Methods
    private synchronized double scaleInput(double val) {
        return (range(pow(val, 3)));
    }

    private synchronized double range(double val) {
        if (val < -1) val = -1;
        if (val > 1) val = 1;
        return val;
    }

    public synchronized boolean isGyroCalibrated() {
        return drive.isGyroCalibrated();
    }

    public void setLightOn() {
        drive.setLightPower(1);
    }

    public void setLightOff() {
        drive.setLightPower(0);
    }

    public void dropTeamMarker() {
        //Teleop dump marker sequence
        if (RobotState.currentMatchState == MatchState.TELEOP) {
            drive.setMarkerServo(DRIVE_TEAM_MARKER_EXTENDED);
            delay(DRIVE_DUMP_TEAM_MARKER_DELAY);
            drive.setMarkerServo(DRIVE_TEAM_MARKER_TELEOP_RETRACTED);
            return;
        }

        //Auton dump marker sequence
        telemetry.addData(INFO, "Start marker dump");
        currentPath.pause();
        telemetry.addData(INFO, "Pause path");
        drive.setMarkerServo(DRIVE_TEAM_MARKER_EXTENDED);
        delay(DRIVE_DUMP_TEAM_MARKER_DELAY);
        drive.setMarkerServo(DRIVE_TEAM_MARKER_RETRACTED);
        currentPath.resume();
        telemetry.addData(INFO, "Marker dumped");
    }

    public void resetPosition() {
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}