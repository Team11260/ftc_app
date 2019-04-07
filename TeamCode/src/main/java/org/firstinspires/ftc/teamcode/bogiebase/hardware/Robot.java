package org.firstinspires.ftc.teamcode.bogiebase.hardware;

import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.ExpansionHubMonitor;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.Potentiometer;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.TensorFlow;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.Path;
import org.firstinspires.ftc.teamcode.framework.util.AbstractRobot;

import java.util.concurrent.Callable;

import static org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry.LogMode.INFO;
import static org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry.LogMode.TRACE;
import static org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition.UNKNOWN;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;
    private TensorFlow tensorFlow;
    private ExpansionHubMonitor hub;

    //Robot Methods
    public Robot() {
        hardware = new HardwareDevices();
        hub = new ExpansionHubMonitor("Expansion Hub 1");

        if (RobotState.currentMatchState == RobotState.MatchState.AUTONOMOUS) {
            telemetry.addData(INFO, "starting tensorflow");
            tensorFlow = new TensorFlow(TensorFlow.CameraOrientation.VERTICAL, "Webcam 1", false);

            RobotState.currentSamplePosition = UNKNOWN;
            telemetry.addData(INFO, "Current Sample Position: " + RobotState.currentSamplePosition);
            telemetry.update();

            setLightOn();
        }
    }


    public void updateSamplePosition(int loop) {

        //Object recognition loop
        if (loop % 5 == 0) tensorFlow.restart();

        SamplePosition currentPosition = tensorFlow.getSamplePosition();

        if (currentPosition != RobotState.currentSamplePosition && currentPosition != UNKNOWN) {
            RobotState.currentSamplePosition = currentPosition;

            telemetry.addData(INFO, "Current Sample Position: " + currentPosition.toString());
            telemetry.update();
        }
    }

    public void stopTensorFlow() {
        setLightOff();
        tensorFlow.stop();
    }

    public void updateAll() {
        if(RobotState.currentMatchState == RobotState.MatchState.TELEOP) {
            hardware.drive.update();
            hardware.intake.update();
            hardware.mineralLift.update();
            hardware.robotLift.update();
        }

        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.getSmartdashboard().putValue("Left drive current", hub.getCurrentDrawMotor0());
        telemetry.getSmartdashboard().putValue("Right drive current", hub.getCurrentDrawMotor1());
        telemetry.getSmartdashboard().putValue("Robot lift current", hub.getCurrentDrawMotor2());
        telemetry.getSmartdashboard().putValue("Mineral lift current", hub.getCurrentDrawMotor3());
        telemetry.getSmartdashboard().putValue("Total current", hub.getTotalCurrentDraw());
        telemetry.getSmartdashboard().putValue("Voltage", hub.getVoltage());
        telemetry.getSmartdashboard().putValue("Heading", hardware.drive.getHeading());
        telemetry.getSmartdashboard().putValue("Pitch", hardware.drive.getPitch());
        telemetry.getSmartdashboard().putValue("Left drive position", hardware.drive.getLeftPosition());
        telemetry.getSmartdashboard().putValue("Right drive position", hardware.drive.getRightPosition());

        telemetry.addDataPhone(TRACE, "Left drive current: " + hub.getCurrentDrawMotor0());
        telemetry.addDataPhone(TRACE, "Right drive current: " + hub.getCurrentDrawMotor1());
        telemetry.addDataPhone(TRACE, "Robot lift current: " + hub.getCurrentDrawMotor2());
        telemetry.addDataPhone(TRACE, "Mineral lift current: " + hub.getCurrentDrawMotor3());
        telemetry.addDataPhone(INFO, "Total current: " + hub.getTotalCurrentDraw());
        telemetry.addDataPhone(INFO, "Voltage: " + hub.getVoltage());

        telemetry.addDataPhone(INFO, "Mineral Lift Position: " + hardware.mineralLift.getMineralLiftPosition());
        telemetry.addDataPhone(INFO, "Mineral Lift Time: " + hardware.mineralLift.getMineralLiftTime());
        telemetry.update();
    }

    public void stop() {
        if (RobotState.currentMatchState == RobotState.MatchState.AUTONOMOUS) stopTensorFlow();
        hardware.stop();
    }

    //Drive Methods
    public void setDriveY(double y) {
        hardware.drive.setY(y);
    }

    public void setDriveZ(double z) {
        hardware.drive.setZ(z);
    }

    public void driveUpdate() {
        hardware.drive.updateYZDrive();
    }

    public void setDrivePower(double l, double r) {
        if(hub.getTotalCurrentDraw() > 18) {
            if(l > 0.3) l = 0.3;
            if(l < -0.3) l = -0.3;
            if(r > 0.3) r = 0.3;
            if(r < -0.3) r = -0.3;
        }
        hardware.drive.setPower(l, r);
    }

    public Callable toggleDriveInvertedCallable() {
        return () -> {
            hardware.drive.toggleInverted();
            return true;
        };
    }

    public void turnTo(double angle, double speed, double error, int period) {
        hardware.drive.turnTo(angle, speed, error, period);
    }

    public void runDrivePath(Path path) {
        hardware.drive.runDrivePath(path);
    }

    public int[][] recordPath(int numSamples, int timeInterval) {
        return hardware.drive.recordPath(numSamples, timeInterval);
    }

    public void runPath(int[] left, int[] right, int timeInterval) {
        hardware.drive.runPath(left, right, timeInterval);
    }

    public void driveTo(double distance, double speed) {
        hardware.drive.driveTo(distance, speed);
    }

    public void driveTo(double distance, double speed, int angle) {
        hardware.drive.driveTo(distance, speed, angle);
    }

    public int[] recordPathWithHeading(int numSamples, int timeInterval) {
        return hardware.drive.recordPathWithHeading(numSamples, timeInterval);
    }

    public void runPathWithHeading(int[] values, int timeInterval, double speed) {
        hardware.drive.runPathWithHeading(values, timeInterval, speed);
    }

    public void setPosition(int position, double power) {
        hardware.drive.setPosition(position, power);
    }

    public boolean isGyroCalibrated() {
        return hardware.drive.isGyroCalibrated();
    }

    public Callable autonReleaseWheelsSequenceCallable() {
        return () -> {
            hardware.drive.autonReleaseWheelsSequence();
            return true;
        };
    }

    public void autonReleaseWheelsSequence() {
        hardware.drive.autonReleaseWheelsSequence();
    }

    public void autonDriveToWallSequence() {
        hardware.drive.autonDriveToWallSequence();
    }

    public Callable autonDriveToWallSequenceCallable() {
        return () -> {
            hardware.drive.autonDriveToWallSequence();
            return true;
        };
    }

    public double getHeading() {
        return hardware.drive.getHeading();
    }

    public double getPitch() {
        return hardware.drive.getPitch();
    }

    public Callable getPitchCallable() {
        return () -> {
            hardware.drive.getPitch();
            return true;
        };
    }

    public void setLightOn() {
        hardware.drive.setLightOn();
    }

    public void setLightOff() {
        hardware.drive.setLightOff();
    }

    //Intake Methods
    public Callable autonIntakeSequenceCallable() {
        return () -> {
            hardware.intake.autonIntakeSequence();
            return true;
        };
    }

    public Callable beginIntakingCallable() {
        return () -> {
            beginIntaking();
            return true;
        };
    }

    public void beginIntaking() {
        hardware.intake.beginIntaking();
    }

    public Callable finishIntakingCallable() {
        return () -> {
            finishIntaking();
            return true;
        };
    }

    public void finishIntaking() {
        hardware.intake.finishIntaking();
    }

    public Callable reverseIntakeCallable() {
        return () -> {
            reverseIntake();
            return true;
        };
    }

    public void reverseIntake() {
        hardware.intake.reverseIntake();
    }

    public void slowReverseIntake() {
        hardware.intake.slowReverseIntake();
    }

    public Callable liftIntakeCallable() {
        return () -> {
            liftIntake();
            return true;
        };
    }

    public void liftIntake() {
        hardware.intake.liftIntake();
    }

    public Callable lowerIntakeCallable() {
        return () -> {
            lowerIntake();
            return true;
        };
    }

    public void lowerIntake() {
        hardware.intake.lowerIntake();
    }

    //Mineral Lift Methods
    public Callable moveMineralLiftToCollectPositionCallable() {
        return () -> {
            moveMineralLiftToCollectPosition();
            return true;
        };
    }

    public void moveMineralLiftToCollectPosition() {
        hardware.mineralLift.closeGate();
        hardware.mineralLift.moveToCollectPosition();
        hardware.drive.setInverted(false);
    }

    public Callable moveMineralLiftToDumpPositionCallable() {
        return () -> {
            moveMineralLiftToDumpPosition();
            //liftIntake();
            return true;
        };
    }

    public void moveMineralLiftToDumpPosition() {
        hardware.mineralLift.moveToDumpPosition();
        hardware.drive.setInverted(true);
    }

    public Callable openMineralGateCallable() {
        return () -> {
            openMineralGate();
            return true;
        };
    }

    public void openMineralGate() {
        hardware.mineralLift.openGate();
    }

    public Callable closeMineralGateCallable() {
        return () -> {
            closeMineralGate();
            return true;
        };
    }

    public void closeMineralGate() {
        hardware.mineralLift.closeGate();
    }

    public Callable toggleMineralGateCallable() {
        return () -> {
            hardware.mineralLift.toggleGate();
            return true;
        };
    }

    public void setAngleServoPositionHorizontal() {
        hardware.mineralLift.setAngleServoPositionHorizontal();
    }

    public Callable setAngleServoPositionHorizontalCallable() {
        return () -> {
            hardware.mineralLift.setAngleServoPositionHorizontal();
            return true;
        };
    }

    public void setAngleServoPositionDump() {
        hardware.mineralLift.setAngleServoPositionDump();
    }

    public Callable setAngleServoPositionDumpCallable() {
        return () -> {
            hardware.mineralLift.setAngleServoPositionDump();
            return true;
        };
    }

    public void setAngleServoPositionVertical() {
        hardware.mineralLift.setAngleServoPositionVertical();
    }

    public Callable setAngleServoPositionVerticalCallable() {
        return () -> {
            hardware.mineralLift.setAngleServoPositionVertical();
            return true;
        };
    }

    public Callable autonLowerMineralLiftSequenceCallable() {
        return () -> {
            autonLowerMineralLiftSequence();
            return true;
        };
    }

    public void autonLowerMineralLiftSequence() {
        hardware.mineralLift.autonLowerLiftSequence();
    }

    public Callable autonMoveMineralLiftToCollectPositionSequenceCallable() {
        return () -> {
            autonMoveMineralLiftToCollectPositionSequence();
            return true;
        };
    }

    public void autonMoveMineralLiftToCollectPositionSequence() {
        hardware.mineralLift.autonMoveToCollectPositionSequence();
    }

    public Callable autonMoveMineralLiftToDumpPositionSequenceCallable() {
        return () -> {
            autonMoveMineralLiftToDumpPositionSequence();
            return true;
        };
    }

    public void autonMoveMineralLiftToDumpPositionSequence() {
        hardware.mineralLift.autonMoveToDumpPositionSequence();
    }

    //robot lift methods
    public Callable robotLiftUpCallable() {
        return () -> {
            hardware.robotLift.robotLiftUp();
            return true;
        };

    }

    public Callable robotLiftDownCallable() {
        return () -> {
            hardware.robotLift.robotLiftDown();
            return true;
        };
    }

    public Callable robotLiftStopCallable() {
        return () -> {
            hardware.robotLift.robotLiftStop();
            return true;
        };
    }

    public Callable moveRobotLiftToTopCallable() {
        return () -> {
            hardware.robotLift.raiseLift();
            return true;
        };
    }

    public Callable moveRobotLiftToBottomCallable() {
        return () -> {
            moveRobotLiftToBottom();
            return true;
        };
    }

    public void moveRobotLiftToBottom() {
        hardware.robotLift.autonLowerLiftSequence();
    }

    public Callable finishRobotLiftToBottomSequenceCallable() {
        return () -> {
            hardware.robotLift.autonFinishLowerLiftSequence();
            return true;
        };
    }

    public Callable dropMarkerCallable() {
        return () -> {
            hardware.drive.dropTeamMarker();
            return true;
        };
    }

    public Callable resetLiftPositionCallable() {
        return () -> {
            hardware.robotLift.resetLiftPosition();
            return true;
        };
    }
}