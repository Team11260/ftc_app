package org.firstinspires.ftc.teamcode.bogiebase.hardware;

import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.ExpansionHubMonitor;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.tensorflow.TensorFlowImpl;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.Path;
import org.firstinspires.ftc.teamcode.framework.util.AbstractRobot;
import org.firstinspires.ftc.teamcode.framework.util.RobotCallable;

import static org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry.LogMode.INFO;
import static org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry.LogMode.TRACE;
import static org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition.UNKNOWN;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;
    private TensorFlowImpl tensorFlow;
    private ExpansionHubMonitor hub;

    //Robot Methods
    public Robot() {
        hardware = new HardwareDevices();
        hub = new ExpansionHubMonitor("Expansion Hub 1");

        if (RobotState.currentMatchState == RobotState.MatchState.AUTONOMOUS) {
            telemetry.addData(INFO, "starting tensorflow");
            tensorFlow = new TensorFlowImpl(TensorFlowImpl.CameraOrientation.VERTICAL, "Webcam 1", false);

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
        if (RobotState.currentMatchState == RobotState.MatchState.TELEOP) {
            hardware.drive.update();
            hardware.intake.update();
            hardware.mineralLift.update();
            hardware.robotLift.update();
        }

        //updateTelemetry();
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
        telemetry.addDataPhone(INFO, "Limit Switch: " + hardware.mineralLift.getBottomLimitSwitchPressed());

        telemetry.addDataPhone(INFO, "Mineral Lift Position: " + hardware.mineralLift.getMineralLiftPosition());
        telemetry.update();
    }

    public double getVoltage() {
        return hub.getVoltage();
    }

    public double getCurrent() {
        return hub.getTotalCurrentDraw();
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

    public void setDrivePowerNoEncoder(double l, double r) {
        hardware.drive.setPowerNoEncoder(l, r);
    }

    public void setDrivePower(double l, double r) {
        hardware.drive.setPower(l, r);
    }

    public void runDrivePath(Path path) {
        hardware.drive.runDrivePath(path);
    }

    public void driveRunPath(org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.Path path) {
        hardware.drive.runPath(path);
    }

    public void setDrivePosition(int position, double power) {
        hardware.drive.setPosition(position, power);
    }

    public int getLeftDrivePosition() {
        return hardware.drive.getLeftPosition();
    }

    public int getRightDrivePosition() {
        return hardware.drive.getRightPosition();
    }

    public void resetDrivePosition() {
        hardware.drive.resetPosition();
    }

    public boolean isGyroCalibrated() {
        return hardware.drive.isGyroCalibrated();
    }

    public RobotCallable autonReleaseWheelsSequenceCallable() {
        return () -> hardware.drive.autonReleaseWheelsSequence();
    }

    public void autonReleaseWheelsSequence() {
        hardware.drive.autonReleaseWheelsSequence();
    }

    public void autonDriveToWallSequence() {
        hardware.drive.autonDriveToWallSequence();
    }

    public RobotCallable autonDriveToWallSequenceCallable() {
        return () -> hardware.drive.autonDriveToWallSequence();
    }

    public double getHeading() {
        return hardware.drive.getHeading();
    }

    public double getPitch() {
        return hardware.drive.getPitch();
    }

    public RobotCallable getPitchCallable() {
        return () -> hardware.drive.getPitch();
    }

    public void setLightOn() {
        hardware.drive.setLightOn();
    }

    public void setLightOff() {
        hardware.drive.setLightOff();
    }

    //Intake Methods
    public RobotCallable autonIntakeSequenceCallable() {
        return () -> hardware.intake.autonIntakeSequence();
    }

    public RobotCallable beginIntakingCallable() {
        return () -> beginIntaking();
    }

    public void beginIntaking() {
        hardware.intake.beginIntaking();
    }

    public RobotCallable finishIntakingCallable() {
        return () -> finishIntaking();
    }

    public void finishIntaking() {
        hardware.intake.finishIntaking();
    }

    public RobotCallable reverseIntakeCallable() {
        return () -> reverseIntake();
    }

    public void reverseIntake() {
        hardware.intake.reverseIntake();
    }

    //Mineral Lift Methods
    public RobotCallable moveMineralLiftToCollectPositionCallable() {
        return () -> moveMineralLiftToCollectPosition();
    }

    public void moveMineralLiftToCollectPosition() {
        hardware.mineralLift.closeGate();
        hardware.mineralLift.moveToCollectPosition();
    }

    public RobotCallable moveMineralLiftToDumpPositionCallable() {
        return () -> moveMineralLiftToDumpPosition();
    }

    public void moveMineralLiftToDumpPosition() {
        hardware.mineralLift.moveToDumpPosition();
    }

    public RobotCallable openMineralGateCallable() {
        return () -> openMineralGate();
    }

    public void openMineralGate() {
        hardware.mineralLift.openGate();
    }

    public RobotCallable closeMineralGateCallable() {
        return () -> closeMineralGate();
    }

    public void closeMineralGate() {
        hardware.mineralLift.closeGate();
    }

    public RobotCallable toggleMineralGateCallable() {
        return () -> hardware.mineralLift.toggleGate();
    }

    public RobotCallable toggleAngleServoTiltAngleCallable() {
        return () -> toggleAngleServoTiltAngle();
    }

    public void toggleAngleServoTiltAngle() {
        hardware.mineralLift.toggleTiltAngle();
    }

    public int getMineralLiftPosition() {
        return hardware.mineralLift.getMineralLiftPosition();
    }

    public void setAngleServoPositionHorizontal() {
        hardware.mineralLift.setAngleServoPositionHorizontal();
    }

    public RobotCallable setAngleServoPositionHorizontalCallable() {
        return () -> hardware.mineralLift.setAngleServoPositionHorizontal();
    }

    public void setAngleServoPositionDump() {
        hardware.mineralLift.setAngleServoPositionDump();
    }

    public RobotCallable setAngleServoPositionDumpCallable() {
        return () -> hardware.mineralLift.setAngleServoPositionDump();
    }

    public void setAngleServoPositionVertical() {
        hardware.mineralLift.setAngleServoPositionVertical();
    }

    public RobotCallable setAngleServoPositionVerticalCallable() {
        return hardware.mineralLift::setAngleServoPositionVertical;
    }

    public RobotCallable autonLowerMineralLiftSequenceCallable() {
        return () -> autonLowerMineralLiftSequence();
    }

    public void autonLowerMineralLiftSequence() {
        hardware.mineralLift.autonLowerLiftSequence();
    }

    public RobotCallable autonMoveMineralLiftToCollectPositionSequenceCallable() {
        return () -> autonMoveMineralLiftToCollectPositionSequence();
    }

    public void autonMoveMineralLiftToCollectPositionSequence() {
        hardware.mineralLift.autonMoveToCollectPositionSequence();
    }

    public RobotCallable autonMoveMineralLiftToDumpPositionSequenceCallable() {
        return () -> autonMoveMineralLiftToDumpPositionSequence();
    }

    public void autonMoveMineralLiftToDumpPositionSequence() {
        hardware.mineralLift.autonMoveToDumpPositionSequence();
    }

    //robot lift methods
    public RobotCallable robotLiftUpCallable() {
        return () -> robotLiftUp();

    }

    public void robotLiftUp() {
        hardware.robotLift.robotLiftUp();
    }

    public RobotCallable robotLiftDownCallable() {
        return () -> robotLiftDown();
    }

    public void robotLiftDown() {
        hardware.robotLift.robotLiftDown();
    }

    public RobotCallable robotLiftStopCallable() {
        return () -> robotLiftStop();
    }

    public void robotLiftStop() {
        hardware.robotLift.robotLiftStop();
    }

    public void moveRobotLiftToBottom() {
        hardware.robotLift.autonLowerLiftSequence();
    }

    public RobotCallable finishRobotLiftToBottomSequenceCallable() {
        return () -> hardware.robotLift.autonFinishLowerLiftSequence();
    }

    public RobotCallable dropMarkerCallable() {
        return () -> hardware.drive.dropTeamMarker();
    }

    public RobotCallable resetLiftPositionCallable() {
        return () -> hardware.robotLift.resetLiftPosition();
    }
}