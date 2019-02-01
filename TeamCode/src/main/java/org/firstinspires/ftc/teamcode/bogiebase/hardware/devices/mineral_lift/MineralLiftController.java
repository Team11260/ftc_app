package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.mineral_lift;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.*;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.*;

public class MineralLiftController extends SubsystemController {

    private MineralLift mineralLift;
    private boolean isMovingDown = false;
    private int[] liftValues = {-1, -1, -1, -1, -1};

    private ElapsedTime cycleTimer, moveTime;

    public MineralLiftController() {
        init();
    }

    public synchronized void init() {
        opModeSetup();

        mineralLift = new MineralLift(hardwareMap);

        cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        moveTime = new ElapsedTime();
        moveTime.reset();
    }

    public synchronized void update() {
        if (isMovingDown) {
            int currentValue = mineralLift.getCurrentPosition();

            if (mineralLift.getPower() == -1 && mineralLift.getCurrentPosition() < MINERAL_LIFT_SLOW_SPEED_TRIGGER_POSITION) {
                mineralLift.setLiftMotorPowerNoEncoder(-0.5);
            }

            if (liftValues[0] == -1) {
                liftValues[0] = 10000;
                liftValues[1] = 0;
                liftValues[2] = -10000;
                liftValues[3] = -20000;
                liftValues[4] = -30000;
                return;
            }
            liftValues[4] = liftValues[3];
            liftValues[3] = liftValues[2];
            liftValues[2] = liftValues[1];
            liftValues[1] = liftValues[0];
            liftValues[0] = currentValue;

            int error = 4;

            if (atPosition(liftValues[0], liftValues[1], error) && atPosition(liftValues[1], liftValues[2], error) &&
                    atPosition(liftValues[2], liftValues[3], error) && atPosition(liftValues[3], liftValues[4], error)) {
                telemetry.addData(DoubleTelemetry.LogMode.INFO, "Mineral lift down finished in: " + moveTime.milliseconds());
                mineralLift.resetPosition();
                liftValues[0] = -1;
                liftValues[1] = -1;
                liftValues[2] = -1;
                liftValues[3] = -1;
                liftValues[4] = -1;
                currentMineralLiftState = MineralLiftState.COLLECT_POSITION;
                isMovingDown = false;
                return;
            }
        }
    }

    public synchronized void stop() {
        mineralLift.stop();
    }

    public synchronized void autonLowerLiftSequence() {
        mineralLift.setTargetPosition(300);
        delay(1000);
        mineralLift.setAngleServoPosition(ANGLE_SERVO_POSITION_HORIZONTAL);

        mineralLift.setCurrentPosition(600);
        mineralLift.setTargetPosition(MINERAL_LIFT_COLLECT_POSITION);

        int currentValue;
        while (AbstractOpMode.isOpModeActive()) {
            currentValue = mineralLift.getCurrentPosition();
            if (liftValues[0] == -1) {
                liftValues[0] = 10000;
                liftValues[1] = 0;
                liftValues[2] = -10000;
                liftValues[3] = -20000;
                return;
            }
            liftValues[3] = liftValues[2];
            liftValues[2] = liftValues[1];
            liftValues[1] = liftValues[0];
            liftValues[0] = currentValue;

            if (atPosition(liftValues[0], liftValues[1], 1) && atPosition(liftValues[0], liftValues[2], 1) && atPosition(liftValues[0], liftValues[3], 1)) {
                telemetry.addData(DoubleTelemetry.LogMode.INFO, "Mineral lift down finished");
                mineralLift.resetPosition();
                liftValues[0] = -1;
                liftValues[1] = -1;
                liftValues[2] = -1;
                liftValues[3] = -1;
                isMovingDown = false;
                return;
            }
        }
    }

    public synchronized void autonMoveToCollectPositionSequence() {
        closeGate();

        mineralLift.setAngleServoPosition(ANGLE_SERVO_POSITION_HORIZONTAL);

        mineralLift.setTargetPosition(MINERAL_LIFT_COLLECT_POSITION);
        delay(500);
        int currentValue;
        while (AbstractOpMode.isOpModeActive()) {
            currentValue = mineralLift.getCurrentPosition();
            if (liftValues[0] == -1) {
                liftValues[0] = 10000;
                liftValues[1] = 0;
                liftValues[2] = -10000;
                liftValues[3] = -20000;
                return;
            }
            liftValues[3] = liftValues[2];
            liftValues[2] = liftValues[1];
            liftValues[1] = liftValues[0];
            liftValues[0] = currentValue;

            if (atPosition(liftValues[0], liftValues[1], 1) && atPosition(liftValues[0], liftValues[2], 1) && atPosition(liftValues[0], liftValues[3], 1)) {
                telemetry.addData(DoubleTelemetry.LogMode.INFO, "Mineral lift down finished");
                mineralLift.resetPosition();
                liftValues[0] = -1;
                liftValues[1] = -1;
                liftValues[2] = -1;
                liftValues[3] = -1;
                isMovingDown = false;
                currentMineralLiftState = MineralLiftState.COLLECT_POSITION;
                return;
            }
        }
    }

    public synchronized void autonMoveToDumpPositionSequence() {
        moveTime.reset();
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Mineral lift up start");
        telemetry.update();
        currentMineralLiftState = MineralLiftState.IN_MOTION;
        //mineralLift.setTargetPosition(MINERAL_LIFT_DUMP_POSITION);
        mineralLift.setLiftMotorPowerNoEncoder(1);
        while (mineralLift.getCurrentPosition() < MINERAL_LIFT_DUMP_POSITION && moveTime.milliseconds() < 3000)
            ;
        currentMineralLiftState = MineralLiftState.DUMP_POSITION;
        mineralLift.setLiftMotorPower(0);
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Mineral up finished in: " + moveTime.milliseconds());
        telemetry.update();
        currentPath.resume();
    }

    public synchronized void moveToCollectPosition() {
        moveTime.reset();
        if (mineralLift.getDistance() < 10) return;
        currentMineralLiftState = MineralLiftState.IN_MOTION;
        //mineralLift.setTargetPosition(MINERAL_LIFT_COLLECT_POSITION);
        mineralLift.setLiftMotorPowerNoEncoder(-1);
        isMovingDown = true;
        mineralLift.setAngleServoPosition(ANGLE_SERVO_POSITION_HORIZONTAL);
        delay(200);
    }

    public synchronized void moveToDumpPosition() {
        moveTime.reset();
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Mineral lift up start");
        telemetry.update();
        currentMineralLiftState = MineralLiftState.IN_MOTION;
        //mineralLift.setTargetPosition(MINERAL_LIFT_DUMP_POSITION);
        mineralLift.setLiftMotorPowerNoEncoder(1);
        delay(200);
        mineralLift.setAngleServoPosition(ANGLE_SERVO_POSITION_VERTICAL);
        while (mineralLift.getCurrentPosition() < MINERAL_LIFT_DUMP_ANGLE_TRIGGER_POSITION && moveTime.milliseconds() < 3000)
            ;
        mineralLift.setAngleServoPosition(ANGLE_SERVO_POSITION_DUMP);
        while (mineralLift.getCurrentPosition() < MINERAL_LIFT_DUMP_POSITION && moveTime.milliseconds() < 3000)
            ;
        currentMineralLiftState = MineralLiftState.DUMP_POSITION;
        mineralLift.setLiftMotorPower(0);
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Mineral up finished in: " + moveTime.milliseconds());
        telemetry.update();
    }

    public synchronized void openGate() {
        mineralLift.setGateServoPosition(MINERAL_GATE_OPEN_POSITION);
        currentMineralGatePosition = MineralGatePosition.OPEN;
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Cycle time: " + cycleTimer.seconds());
        cycleTimer.reset();
    }

    public synchronized void closeGate() {
        mineralLift.setGateServoPosition(MINERAL_GATE_CLOSED_POSITION);
        currentMineralGatePosition = MineralGatePosition.CLOSED;
    }

    public synchronized void toggleGate() {
        if (currentMineralGatePosition == MineralGatePosition.OPEN) closeGate();
        else if (currentMineralLiftState == MineralLiftState.DUMP_POSITION) openGate();
    }

    public void setAngleServoPositionDump() {
        mineralLift.setAngleServoPosition(Constants.ANGLE_SERVO_POSITION_DUMP);
    }

    public void setAngleServoPositionHorizontal() {
        mineralLift.setAngleServoPosition(Constants.ANGLE_SERVO_POSITION_HORIZONTAL);
    }

    public void setAngleServoPositionVertical() {
        mineralLift.setAngleServoPosition(Constants.ANGLE_SERVO_POSITION_VERTICAL);
    }


    public synchronized void setAngleServoPosition(double position) {
        mineralLift.setAngleServoPosition(position);
    }

    private boolean atPosition(double x, double y, double error) {
        double upperRange = x + error;
        double lowerRange = x - error;

        return y >= lowerRange && y <= upperRange;
    }
}
