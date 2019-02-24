package org.firstinspires.ftc.teamcode.skidsteerbase.hardware.devices.arm;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;
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
        telemetry.addData(DoubleTelemetry.LogMode.INFO,"arm position:"+ arm.getRotateMotorPosition());
        telemetry.update();
    }

    @Override
    public synchronized void stop() {

    }

    public void moveToDumpPosition(){
        arm.setRotateMotorPosition(ARM_DUMP_POSITION);

    }

    public void moveToCollectPosition(){
        arm.setRotateMotorPosition(ARM_COLLECT_POSITION);
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
        arm.setGateServoPosition(GATE_OPEN_POSITION);
        currentMineralGatePosition = MineralGatePosition.OPEN;
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Cycle time: " + cycleTimer.seconds());
        cycleTimer.reset();
        //mineralLift.setAngleServoPosition(MINERAL_LIFT_ANGLE_SERVO_DUMP_POSITION);

    }

    public synchronized void closeGate() {
        arm.setGateServoPosition(GATE_CLOSED_POSITION);
        currentMineralGatePosition = MineralGatePosition.CLOSED;
    }

    public synchronized void toggleGate() {
        if (currentMineralGatePosition == RobotState.MineralGatePosition.OPEN) closeGate();
        else if (currentMineralLiftState == MineralLiftState.DUMP_POSITION) openGate();
    }

}
