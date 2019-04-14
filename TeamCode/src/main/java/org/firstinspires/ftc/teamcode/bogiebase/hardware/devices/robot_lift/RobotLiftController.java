package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.robot_lift;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.ROBOT_LIFT_AUTON_DELAY;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.ROBOT_LIFT_LOWERED_POSITION;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.ROBOT_LIFT_LOWER_POWER;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.ROBOT_LIFT_PAWL_ENGAGED;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.ROBOT_LIFT_PAWL_RELEASED;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.ROBOT_LIFT_RELEASE_PAWL_POSITION;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.RobotLiftState;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.currentRobotLiftState;

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
        robotLift.setLiftPower(0);
        robotLift.setServoPosition(ROBOT_LIFT_PAWL_ENGAGED);
    }

    public synchronized void robotLiftDown() {
        robotLift.setPosition(robotLift.getCurrentPosition() + 100);
        robotLift.setServoPosition(ROBOT_LIFT_PAWL_RELEASED);
        delay(300);
        robotLift.setLiftPower(-0.5);
    }

    public synchronized void autonLowerLiftSequence() {
        currentRobotLiftState = RobotLiftState.IN_MOTION;

        delay(ROBOT_LIFT_AUTON_DELAY);

        robotLift.setPosition(ROBOT_LIFT_RELEASE_PAWL_POSITION);
        robotLift.setServoPosition(ROBOT_LIFT_PAWL_RELEASED);

        delay(ROBOT_LIFT_AUTON_DELAY);

        robotLift.setLiftNoEncoderPower(ROBOT_LIFT_LOWER_POWER);

        while (AbstractOpMode.isOpModeActive() && (robotLift.getCurrentPosition() >= ROBOT_LIFT_LOWERED_POSITION)) ;

        robotLift.setPosition(ROBOT_LIFT_LOWERED_POSITION);
    }

    public synchronized void autonFinishLowerLiftSequence() {
        robotLift.setLiftPower(0);
        robotLift.setServoPosition(ROBOT_LIFT_PAWL_ENGAGED);
        currentRobotLiftState = RobotLiftState.LOWERED;
    }

    public synchronized void resetLiftPosition() {
        robotLift.setPosition(0);
        delay(1000);
        robotLift.setLiftPower(0);
    }
}
