package org.firstinspires.ftc.teamcode.skidsteerbase.hardware.devices.arm;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userhardware.trajectories.TrapezoidTrajectory;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;
import org.firstinspires.ftc.teamcode.skidsteerbase.hardware.Constants;
import org.firstinspires.ftc.teamcode.skidsteerbase.hardware.RobotState;

import static org.firstinspires.ftc.teamcode.skidsteerbase.hardware.RobotState.*;
import static org.firstinspires.ftc.teamcode.skidsteerbase.hardware.Constants.*;

public class ArmController extends SubsystemController {

    private Arm arm;

    private ElapsedTime cycleTimer;

    public ArmController() {
        init();
    }

    @Override
    public synchronized void init() {
        opModeSetup();

        arm = new Arm(hardwareMap);

        cycleTimer = new ElapsedTime();
        cycleTimer.reset();

    }

    @Override
    public synchronized void update() {
        /*telemetry.addData(DoubleTelemetry.LogMode.INFO,"arm position:"+ arm.getRotateMotorPosition());
        telemetry.update();*/
    }

    @Override
    public synchronized void stop() {

    }

    public synchronized void moveToDumpPosition(){

        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Starting move to dump position");
        telemetry.update();

        arm.setAngleServoPosition(ARM_ANGLE_HOLD);

        if(currentMineralLiftState == MineralLiftState.COLLECT_POSITION) {

            currentMineralLiftState = MineralLiftState.IN_MOTION;

            int startPosition = arm.getRotateMotorPosition();

            TrapezoidTrajectory trajectory = new TrapezoidTrajectory(ARM_DUMP_POSITION - startPosition, 0.01, 0.001, 1);

            while (!trajectory.isDoneForPosition(arm.getRotateMotorPosition())) {
                arm.setRotateMotorPower(trajectory.velocityForDistance(arm.getRotateMotorPosition()));
                telemetry.addData(DoubleTelemetry.LogMode.INFO, trajectory.velocityForDistance(arm.getRotateMotorPosition()));
                telemetry.update();
            }
        }

        arm.setRotateMotorPosition(ARM_DUMP_POSITION);
        currentMineralLiftState = MineralLiftState.DUMP_POSITION;

        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Done move to dump position");
        telemetry.update();
    }

    public synchronized void moveToCollectPosition(){

        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Starting move to collect position");
        telemetry.update();

        arm.setAngleServoPosition(ARM_ANGLE_INTAKE);

        if(currentMineralLiftState == MineralLiftState.DUMP_POSITION) {
            currentMineralLiftState = MineralLiftState.IN_MOTION;

            int startPosition = arm.getRotateMotorPosition();

            TrapezoidTrajectory trajectory = new TrapezoidTrajectory(startPosition - ARM_COLLECT_POSITION, 0.01, 0.00001, 1);

            while (!trajectory.isDoneForPosition(startPosition - arm.getRotateMotorPosition())) {
                arm.setRotateMotorPower(-trajectory.velocityForDistance(startPosition - arm.getRotateMotorPosition()));
                telemetry.addData(DoubleTelemetry.LogMode.INFO, -trajectory.velocityForDistance(startPosition - arm.getRotateMotorPosition()));
                telemetry.update();
            }
        }

        //arm.setRotateMotorPower(0.0);

        //while (arm.getRotateMotorPosition() > 500 && opModeIsActive());

        arm.setRotateMotorPosition(500);
        currentMineralLiftState = MineralLiftState.COLLECT_POSITION;

        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Done move to collect position");
        telemetry.update();
    }

    public synchronized void beginIntaking() {
        arm.setIntakeServoPosition(INTAKE_FORWARD_POWER);
    }

    public synchronized void finishIntaking() {
        arm.setIntakeServoPosition(INTAKE_STOP_POWER);
    }

    public synchronized void reverseIntake() {
        arm.setIntakeServoPosition(INTAKE_REVERSE_POWER);
    }

    public synchronized void openGate() {
        /*arm.setGateServoPosition(GATE_OPEN_POSITION);
        currentMineralGatePosition = MineralGatePosition.OPEN;
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Cycle time: " + cycleTimer.seconds());
        cycleTimer.reset();*/

        arm.setAngleServoPosition(Constants.ARM_ANGLE_DUMP);
        currentMineralGatePosition = MineralGatePosition.OPEN;
    }

    public synchronized void closeGate() {
        /*arm.setGateServoPosition(GATE_CLOSED_POSITION);
        currentMineralGatePosition = MineralGatePosition.CLOSED;*/

        arm.setAngleServoPosition(Constants.ARM_ANGLE_HOLD);
        currentMineralGatePosition = MineralGatePosition.CLOSED;
    }

    public synchronized void toggleGate() {
        if (currentMineralGatePosition == RobotState.MineralGatePosition.OPEN) closeGate();
        else if (currentMineralLiftState == MineralLiftState.DUMP_POSITION) openGate();
    }
}
