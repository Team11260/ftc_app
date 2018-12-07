package org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.devices.mineral_lift;

import org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.Constants;
import org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

public class MineralLiftController extends SubsystemController {

    private MineralLift mineralLift;
    private boolean isMovingDown = false;
    private int[] liftValues = {-1, -1, -1};

    public MineralLiftController() {
        init();
    }

    @Override
    public synchronized void init() {
        opModeSetup();

        mineralLift = new MineralLift(hardwareMap);
    }

    public synchronized void moveToCollectPosition() {
        RobotState.currentMineralLiftState = RobotState.MineralLiftState.IN_MOTION;
        mineralLift.setCurrentPosition(Constants.MINERAL_LIFT_COLLECT_POSITION);
        isMovingDown = true;
        //while (mineralLift.isLiftInProgress());
        RobotState.currentMineralLiftState = RobotState.MineralLiftState.COLLECT_POSITION;
    }

    public synchronized void moveToDumpPosition() {
        RobotState.currentMineralLiftState = RobotState.MineralLiftState.IN_MOTION;
        mineralLift.setCurrentPosition(Constants.MINERAL_LIFT_DUMP_POSITION);
        //while (mineralLift.isLiftInProgress());
        RobotState.currentMineralLiftState = RobotState.MineralLiftState.DUMP_POSITION;
    }

    public synchronized void update() {
        if (isMovingDown) {
            int currentValue = mineralLift.getCurrentPosition();
            if (liftValues[0] == -1) {
                liftValues[0] = currentValue;
                liftValues[1] = currentValue;
                liftValues[2] = currentValue;
                return;
            }
            liftValues[2]=liftValues[1];
            liftValues[1]=liftValues[0];
            liftValues[0]=currentValue;
            if (atPosition(liftValues[0],liftValues[1],0)&&atPosition(liftValues[1],liftValues[2],0)&&mineralLift.getCurrentPosition()<50) {
                mineralLift.resetPosition();
                liftValues[0] = -1;
                liftValues[1] = -1;
                liftValues[2] = -1;
                isMovingDown = false;
                return;
            }
        }
    }

    public synchronized void openGate() {
        mineralLift.setGateServoPosition(Constants.MINERAL_GATE_OPEN_POSITION);
        RobotState.currentMineralGatePosition = RobotState.MineralGatePosition.OPEN;
    }

    public synchronized void closeGate() {
        mineralLift.setGateServoPosition(Constants.MINERAL_GATE_CLOSED_POSITION);
        RobotState.currentMineralGatePosition = RobotState.MineralGatePosition.CLOSED;
    }

    @Override
    public synchronized void stop() {
        mineralLift.stop();
    }

    private boolean atPosition(double x, double y, double error) {
        double upperRange = x + error;
        double lowerRange = x - error;

        return y >= lowerRange && y <= upperRange;
    }
}
