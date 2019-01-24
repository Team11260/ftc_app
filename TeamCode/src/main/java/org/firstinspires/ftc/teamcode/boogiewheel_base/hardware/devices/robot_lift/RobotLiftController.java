package org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.devices.robot_lift;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.framework.userHardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

import static org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.Constants.*;
import static org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.RobotState.*;

public class RobotLiftController extends SubsystemController {

    private RobotLift robotLift;

    public RobotLiftController() {
        init();
    }

    public synchronized void init() {
        opModeSetup();

        robotLift = new RobotLift(hardwareMap);
    }

    public synchronized void update() {

    }

    public synchronized void stop() {
        robotLift.stop();
    }

    public synchronized void robotLiftUp() {
        robotLift.setLiftPower(1);
    }

    public synchronized void robotLiftStop() {
        // robot is hanging
        robotLift.setLiftPower(0);
        robotLift.setServoPosition(ROBOT_LIFT_PAWL_ENGAGED);
    }

    public synchronized void robotLiftDown() {
        robotLift.setPosition(robotLift.getTargetPosition() + ROBOT_LIFT_RELEASE_PAWL_POSITION);
        robotLift.setServoPosition(ROBOT_LIFT_PAWL_RELEASED);
        delay(1000);
        robotLift.setLiftPower(-0.5);
    }

    public synchronized void raiseLift() {
        robotLift.setPosition(ROBOT_LIFT_RAISED_POSITION);
        currentRobotLiftState = RobotLiftState.RAISED;
    }

    public synchronized void autonLowerLiftSequence() {
        currentRobotLiftState = RobotLiftState.IN_MOTION;

        delay(600);

        robotLift.setPosition(ROBOT_LIFT_RELEASE_PAWL_POSITION);
        robotLift.setServoPosition(ROBOT_LIFT_PAWL_RELEASED);

        delay(600);

        robotLift.setLiftNoEncoderPower(ROBOT_LIFT_LOWER_POWER);

        while (AbstractOpMode.isOpModeActive() && (robotLift.getCurrentPosition() >= ROBOT_LIFT_LOWERED_POSITION));

        robotLift.setPosition(ROBOT_LIFT_LOWERED_POSITION);
    }

    public synchronized void autonFinishLowerLiftSequence() {
        robotLift.setLiftPower(0);
        robotLift.setServoPosition(ROBOT_LIFT_PAWL_ENGAGED);
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "encoder: " + robotLift.getCurrentPosition());
        currentRobotLiftState = RobotLiftState.LOWERED;
    }

    public synchronized void resetLiftPos() {
        robotLift.setPosition(0);
        delay(5000);
        robotLift.setLiftPower(0);
    }
}
