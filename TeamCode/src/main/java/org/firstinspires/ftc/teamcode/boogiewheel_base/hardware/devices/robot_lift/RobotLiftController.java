package org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.devices.robot_lift;

import org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.Constants;
import org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

public class RobotLiftController extends SubsystemController {

    private RobotLift robotLift;

    public RobotLiftController() {
        init();
    }

    @Override
    public synchronized void init() {
        opModeSetup();

        robotLift = new RobotLift(hardwareMap);
    }

    public synchronized void robotLiftUp() {
        robotLift.setLiftPower(1);
    }

    public synchronized void robotLiftStop() {
        robotLift.setLiftPower(0);
    }

    public synchronized void robotLiftDown() {
        robotLift.setLiftPower(-0.5);
    }

    public synchronized void raiseLift() {
        robotLift.setPosition((Constants.ROBOT_LIFT_RAISED_POSITION));
        RobotState.currentRobotLiftState = RobotState.RobotLiftState.RAISED;
    }

    public synchronized void lowerLift() {
        robotLift.setPosition((Constants.ROBOT_LIFT_LOWERED_POSITION));
        RobotState.currentRobotLiftState = RobotState.RobotLiftState.LOWERED;
    }

    @Override
    public synchronized void stop() {
        robotLift.stop();
    }
}
