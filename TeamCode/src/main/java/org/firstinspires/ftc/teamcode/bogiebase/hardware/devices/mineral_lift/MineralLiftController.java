package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.mineral_lift;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.*;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.*;

public class MineralLiftController extends SubsystemController {

    private MineralLift mineralLift;

    private boolean steepTilt = false;

    private ElapsedTime cycleTimer, moveTime;

    public MineralLiftController() {
        init();
    }

    public synchronized void init() {
        opModeSetup();

        mineralLift = new MineralLift(hardwareMap);

        steepTilt = telemetry.getBoolean("teleop_position", false);

        cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        moveTime = new ElapsedTime();
        moveTime.reset();
    }

    public void update() {

    }

    public synchronized void stop() {
        mineralLift.stop();
    }

    public synchronized void autonLowerLiftSequence() {
        mineralLift.setTargetPosition(MINERAL_LIFT_AUTON_RAISED_POSITION);

        delay(500);

        mineralLift.setAngleServoPosition(MINERAL_LIFT_ANGLE_SERVO_HORIZONTAL_POSITION);

        delay(1000);

        mineralLift.setLiftMotorPowerNoEncoder(-MINERAL_LIFT_AUTON_SPEED);

        delay(1000);

        mineralLift.resetPosition();
    }

    public synchronized void autonMoveToCollectPositionSequence() {
        moveTime.reset();

        closeGate();

        mineralLift.setAngleServoPosition(MINERAL_LIFT_ANGLE_SERVO_HORIZONTAL_POSITION);

        mineralLift.setLiftMotorPowerNoEncoder(-MINERAL_LIFT_AUTON_SPEED);

        delay(2000);

        mineralLift.resetPosition();
    }

    public synchronized void autonMoveToDumpPositionSequence() {
        currentPath.pause();
        moveTime.reset();

        currentMineralLiftState = MineralLiftState.IN_MOTION;
        mineralLift.setAngleServoPosition(MINERAL_LIFT_ANGLE_SERVO_STEEP_DUMP_POSITION);
        mineralLift.setLiftMotorPowerNoEncoder(1);
        while (mineralLift.getCurrentPosition() < MINERAL_LIFT_DUMP_POSITION && moveTime.milliseconds() < 3000 && opModeIsActive()) ;
        currentMineralLiftState = MineralLiftState.DUMP_POSITION;

        mineralLift.setLiftMotorPowerNoEncoder(0.8);
        delay(500);

        mineralLift.setLiftMotorPower(0);

        currentPath.resume();
    }

    public synchronized void moveToCollectPosition() {
        moveTime.reset();
        if (mineralLift.getDistance() < 10) return;
        currentMineralLiftState = MineralLiftState.IN_MOTION;
        mineralLift.setLiftMotorPowerNoEncoder(-MINERAL_LIFT_FULL_SPEED);
        mineralLift.setAngleServoPosition(MINERAL_LIFT_ANGLE_SERVO_HORIZONTAL_POSITION);

        while (mineralLift.getCurrentPosition() > MINERAL_LIFT_SLOW_SPEED_TRIGGER_POSITION && !getBottomLimitSwitchPressed() && moveTime.milliseconds() < 2000 && opModeIsActive());

        mineralLift.setLiftMotorPowerNoEncoder(-MINERAL_LIFT_SLOW_SPEED);

        while (!getBottomLimitSwitchPressed() && moveTime.milliseconds() < 2000 && opModeIsActive());

        delay(500);

        mineralLift.resetPosition();
        currentMineralLiftState = MineralLiftState.COLLECT_POSITION;
    }

    public synchronized void moveToDumpPosition() {
        moveTime.reset();
        currentMineralLiftState = MineralLiftState.IN_MOTION;

        mineralLift.setGateServoPosition(MINERAL_LIFT_GATE_HOLD_POSITION);

        mineralLift.setLiftMotorPowerNoEncoder(MINERAL_LIFT_FULL_SPEED);
        mineralLift.setAngleServoPosition(MINERAL_LIFT_ANGLE_SERVO_VERTICAL_POSITION);
        while (mineralLift.getCurrentPosition() < MINERAL_LIFT_DUMP_ANGLE_TRIGGER_POSITION && moveTime.milliseconds() < 2000 && opModeIsActive()) ;
        setAngleServoPositionDump();
        while (mineralLift.getCurrentPosition() < MINERAL_LIFT_DUMP_POSITION && moveTime.milliseconds() < 2000 && opModeIsActive());
        mineralLift.setLiftMotorPowerNoEncoder(0.8);
        delay(500);
        currentMineralLiftState = MineralLiftState.DUMP_POSITION;
        mineralLift.setLiftMotorPowerNoEncoder(0);
    }

    public int getMineralLiftPosition() {
        return mineralLift.getCurrentPosition();
    }

    public synchronized void openGate() {
        mineralLift.setGateServoPosition(MINERAL_LIFT_GATE_CLOSED_POSITION);
        delay(250);
        mineralLift.setGateServoPosition(MINERAL_LIFT_GATE_OPEN_POSITION);
        currentMineralGatePosition = MineralGatePosition.OPEN;
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Cycle time: " + cycleTimer.seconds());
        cycleTimer.reset();
    }

    public synchronized void closeGate() {
        mineralLift.setGateServoPosition(MINERAL_LIFT_GATE_CLOSED_POSITION);
        currentMineralGatePosition = MineralGatePosition.CLOSED;
        //mineralLift.setAngleServoPosition(MINERAL_LIFT_ANGLE_SERVO_HORIZONTAL_POSITION);
    }

    public synchronized void toggleGate() {
        if (currentMineralGatePosition == MineralGatePosition.OPEN) closeGate();
        else if (currentMineralLiftState == MineralLiftState.DUMP_POSITION) openGate();
    }

    public synchronized void toggleTiltAngle() {
        steepTilt = !steepTilt;
    }

    public synchronized void setAngleServoPositionDump() {
        mineralLift.setAngleServoPosition(steepTilt ? Constants.MINERAL_LIFT_ANGLE_SERVO_STEEP_DUMP_POSITION : Constants.MINERAL_LIFT_ANGLE_SERVO_SHALLOW_DUMP_POSITION);
    }

    public synchronized void setAngleServoPositionHorizontal() {
        mineralLift.setAngleServoPosition(Constants.MINERAL_LIFT_ANGLE_SERVO_HORIZONTAL_POSITION);
    }

    public synchronized void setAngleServoPositionVertical() {
        mineralLift.setAngleServoPosition(Constants.MINERAL_LIFT_ANGLE_SERVO_VERTICAL_POSITION);
    }

    public boolean getBottomLimitSwitchPressed() {
        return mineralLift.getBottomLimitSwitchPressed();
    }

    public synchronized void setAngleServoPosition(double position) {
        mineralLift.setAngleServoPosition(position);
    }
}
