package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userhardware.PIDController;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.ExpansionHubMonitor;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.DriveSegment;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.Path;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.Segment;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.TurnSegment;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

import java.text.DecimalFormat;

import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.*;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.*;
import static org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry.LogMode.INFO;

public class DriveController extends SubsystemController {

    private Drive drive;

    private PIDController anglePID, straightPID, distancePID;

    private double baseHeading = 0;

    private double turnY = 0, turn_z = 0, leftPower = 0, rightPower = 0, Drive_Power = 1.0;

    public ElapsedTime runtime;

    private DecimalFormat DF;

    //Utility Methods
    public DriveController() {
        init();
    }

    public synchronized void init() {

        opModeSetup();

        runtime = new ElapsedTime();

        DF = new DecimalFormat("#.##");

        //Put general setup here
        drive = new Drive(hardwareMap);
        anglePID = new PIDController(15, 0.1, 150, 0.3, 0.08);
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
    public synchronized void turnTo(double angle, double speed, double error, int period) {
        AbstractOpMode.delay(100);

        baseHeading = angle;

        telemetry.addData("starting turn segment---------------------------");
        anglePID.reset();
        anglePID.setMinimumOutput(0);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double power;
        while (opModeIsActive()) {
            //While we are not in the error band keep turning
            while (!atPosition(angle, drive.getHeading(), error) && opModeIsActive()) {
                telemetry.addDataDB("--------------------");
                //Use the PIDController class to calculate power values for the wheels
                if (angle - getHeading() > 180) {
                    power = anglePID.output(angle, 180 + (180 + getHeading()));
                    telemetry.addData("How Far", 180 + (180 + getHeading()));
                } else {
                    power = anglePID.output(angle, getHeading());
                    telemetry.addData("How Far", getHeading());
                }
                setPower(-power * speed, power * speed);
                telemetry.addData("Heading", getHeading());
                telemetry.addData("Power", power);
                telemetry.update();
            }
            runtime.reset();
            while (runtime.milliseconds() < period) {
                if ((abs(getHeading() - angle)) > error && (abs(getHeading() + angle)) > error)
                    break;
            }
            if ((abs(getHeading() - angle)) > error && (abs(getHeading() + angle)) > error)
                continue;
            telemetry.addData("Final heading", getHeading());
            telemetry.update();
            drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return;
        }
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public synchronized void driveTo(double distance, double speed) {
        driveTo(distance, speed, (int) baseHeading);
    }

    public synchronized void driveTo(double distance, double speed, int angle) {
        baseHeading = angle;

        AbstractOpMode.delay(100);

        telemetry.addData("starting drive segment---------------------------");
        straightPID.reset(); //Resets the PID values in the PID class to make sure we do not have any left over values from the last segment
        distancePID.reset();
        straightPID.setMinimumOutput(0);
        int position = (int) (distance * DRIVE_COUNTS_PER_INCH); //
        telemetry.addData("Encoder counts: " + position);
        double turn;
        speed = range(speed);
        telemetry.addData("Speed: " + speed);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPosisionP(5);
        double startHeading = angle;
        telemetry.addData("Start Heading: " + startHeading);
        telemetry.update();
        double leftPower, rightPower;
        double power;
        while ((!atPosition(position, drive.getLeftPosition(), 15) && !atPosition(position, drive.getRightPosition(), 15)) && opModeIsActive()) {
            power = range(distancePID.output(position, drive.getRightPosition()));

            turn = straightPID.output(startHeading, getHeading());

            if (power > 0) {
                leftPower = range(power * (speed - turn));
                rightPower = range(power * (speed + turn));
            } else {
                leftPower = range(power * (speed + turn));
                rightPower = range(power * (speed - turn));
            }

            drive.setPower(leftPower, rightPower);

            telemetry.addData("Encoder counts: " + position);
            telemetry.addData("Left Position: " + drive.getLeftPosition());
            telemetry.addData("Right Position: " + drive.getRightPosition());
            telemetry.addData("Left Power: " + leftPower);
            telemetry.addData("Right Power: " + rightPower);
            telemetry.addData("Heading: " + getHeading());
            telemetry.addData("PID Output: " + turn);
            telemetry.update();
        }

        for (int i = 0; i < 5; i++) {
            power = range(distancePID.output(position, (drive.getRightPosition() + drive.getLeftPosition()) / 2));

            turn = straightPID.output(startHeading, getHeading());

            if (power > 0) {
                leftPower = range(power * (speed - turn));
                rightPower = range(power * (speed + turn));
            } else {
                leftPower = range(power * (speed + turn));
                rightPower = range(power * (speed - turn));
            }

            drive.setPower(leftPower, rightPower);

            telemetry.addData("Encoder counts: " + position);
            telemetry.addData("Left Position: " + drive.getLeftPosition());
            telemetry.addData("Right Position: " + drive.getRightPosition());
            telemetry.addData("Left Power: " + leftPower);
            telemetry.addData("Right Power: " + rightPower);
            telemetry.addData("Heading: " + getHeading());
            telemetry.update();
        }

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public synchronized void runDrivePath(Path path) {

        boolean lastPathPaused = false;

        if (currentPath != null && currentPath.isPaused()) {
            lastPathPaused = true;
        }

        currentPath = path;
        currentPath.reset();

        if(lastPathPaused) currentPath.pause();

        telemetry.addData(INFO, "Starting path: " + currentPath.getName() + "  paused: " + currentPath.isPaused() + "  done: " + currentPath.isDone());

        while (!path.isDone() && opModeIsActive()) {

            //Path is done
            if (path.getNextSegment() == null) break;

            telemetry.addData(INFO, "Starting segment: " + path.getCurrentSegment().getName() + " in path: " + currentPath.getName() + "  paused: " + currentPath.isPaused() + "  done: " + currentPath.isDone());

            if (path.getCurrentSegment().getType() == Segment.SegmentType.TURN) {
                turnToSegment((TurnSegment) path.getCurrentSegment());
            } else if (path.getCurrentSegment().getType() == Segment.SegmentType.DRIVE) {
                driveToSegment((DriveSegment) path.getCurrentSegment());
            }
        }
    }

    public synchronized void turnToSegment(TurnSegment segment) {

        double angle = segment.getAngle(), speed = segment.getSpeed(), error = segment.getError(), period = segment.getPeriod();

        baseHeading = angle;

        anglePID.reset();
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double power;
        while (opModeIsActive()) {

            double currentHeading;

            int loop = 0;
            runtime.reset();

            //While we are not in the error band keep turning
            while (!atPosition(angle, currentHeading = getHeading(), error) && (angle + error > 180 ? !atPosition((((angle + error) - 180) - 180) - error, currentHeading, error) : true) && (angle - error < -180 ? !atPosition((((angle - error) + 180) + 180) + error, currentHeading, error) : true) && opModeIsActive()) {

                if (segment.isDone()) {
                    setPower(0, 0);
                    return;
                }
                if (!segment.isRunning()) {
                    setPower(0, 0);
                    continue;
                }

                //Use the PIDController class to calculate power values for the wheels
                if (angle - currentHeading > 180) {
                    power = anglePID.output(angle, 360 + currentHeading);
                } else if (currentHeading - angle > 180) {
                    power = anglePID.output(angle, angle - (360 - (currentHeading - angle)));
                } else {
                    power = anglePID.output(angle, currentHeading);
                }
                setPower(-power * speed, power * speed);

                loop++;
            }

            while (runtime.milliseconds() < period) {
                if ((abs(getHeading() - angle)) > error && (abs(getHeading() + angle)) > error)
                    break;
            }
            if ((abs(getHeading() - angle)) > error && (abs(getHeading() + angle)) > error)
                continue;


            telemetry.addData(INFO, "Average loop time for turn: " + runtime.milliseconds() / loop);
            telemetry.addData(INFO, "Left encoder position: " + drive.getLeftPosition() + "  Right encoder position: " + drive.getRightPosition());
            telemetry.addData(INFO, "Final angle: " + getHeading());
            telemetry.update();

            drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return;
        }
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public synchronized void driveToSegment(DriveSegment segment) {

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
        drive.setPosisionP(5);
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
        /*int[] values = new int[5];
        for (int i = 0; i < 5; i++) {
            values[i] = i * 1000000;
        }

        int error = 3;
        while (RobotState.currentPath.getCurrentSegment().getName().equals("drive to wall") && (!atPosition(values[0], values[1], error) ||
                !atPosition(values[1], values[2], error) || !atPosition(values[2], values[3], error) || !atPosition(values[3], values[4], error)) && opModeIsActive()) {
            for (int i = 4; i > 0; i--) {
                values[i] = values[i - 1];
            }
            values[0] = (drive.getLeftPosition() + drive.getRightPosition()) / 2;
        }

        if (!RobotState.currentPath.getCurrentSegment().getName().equals("drive to wall")) return;

        RobotState.currentPath.nextSegment();*/

        while (RobotState.currentPath.getCurrentSegment().getName().equals("drive to wall") && opModeIsActive() && drive.getLeftMotorCurrentDraw()<6000 && drive.getRightMotorCurrentDraw()<6000) ;

        if (!RobotState.currentPath.getCurrentSegment().getName().equals("drive to wall")) return;

        RobotState.currentPath.nextSegment();
    }

    public synchronized double getHeading() {
        return drive.getHeading();
    }

    public synchronized void resetAngleToZero() {
        drive.resetAngleToZero();
    }

    //TeleOp Methods
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

    public synchronized void setInverted(boolean inverted) {
        if (inverted) currentDriveDirection = DriveDirection.REVERSED;
        else currentDriveDirection = DriveDirection.FORWARD;
    }

    public synchronized void toggleInverted() {
        if (currentDriveDirection == DriveDirection.FORWARD)
            currentDriveDirection = DriveDirection.REVERSED;
        else currentDriveDirection = DriveDirection.FORWARD;
    }

    //Util Methods
    public synchronized int[][] recordPath(int numSamples, int timeInterval) {
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int[][] values = new int[2][numSamples];
        runtime.reset();
        for (int i = 0; i < numSamples; i++) {
            while (runtime.milliseconds() < timeInterval && opModeIsActive()) ;
            values[0][i] = drive.getLeftPosition();
            values[1][i] = drive.getRightPosition();
            if (!opModeIsActive()) break;
            runtime.reset();
        }
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return values;
    }

    public synchronized void runPath(int[] left, int[] right, int timeInterval) {
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.setPower(1, 1);
        runtime.reset();
        for (int i = 0; i < (left.length - right.length <= 0 ? right.length : left.length); i++) {
            while (runtime.milliseconds() < timeInterval && opModeIsActive()) ;
            drive.setTargetPosition(left[i], right[i]);
            if (!opModeIsActive()) break;
            runtime.reset();
        }
    }

    public synchronized int[] recordPathWithHeading(int numSamples, int timeInterval) {
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int[] values = new int[numSamples * 3];
        runtime.reset();
        for (int i = 0; i < numSamples * 3; i += 3) {
            while (runtime.milliseconds() < timeInterval && opModeIsActive()) ;
            telemetry.addData(drive.getHeading());
            telemetry.update();
            values[i] = (int) drive.getHeading();
            values[i + 1] = drive.getLeftPosition();
            values[i + 2] = drive.getRightPosition();
            if (!opModeIsActive()) break;
            runtime.reset();
        }
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return values;
    }

    public synchronized void runPathWithHeading(int[] values, int timeInterval, double speed) {
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.setPower(1, 1);
        runtime.reset();
        double turn;
        for (int i = 0; i < values.length; i += 3) {
            while (runtime.milliseconds() < timeInterval && opModeIsActive()) ;
            turn = anglePID.output(values[i], getHeading());
            drive.setPower(speed - turn, speed + turn);
            drive.setTargetPosition(values[i + 1], values[i + 2]);
            if (!opModeIsActive()) break;
            runtime.reset();
        }
    }

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
}