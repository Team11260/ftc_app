package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.intake;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

import static org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants.*;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState.*;

public class IntakeController extends SubsystemController {

    private Intake intake;

    public IntakeController() {
        init();
    }

    public synchronized void init() {

        opModeSetup();

        intake = new Intake(hardwareMap);
    }

    public synchronized void update() {

    }

    public synchronized void stop() {
        intake.stop();
    }

    public synchronized void autonIntakeSequence() {
        while ((!currentPath.getCurrentSegment().getName().equals("drive to minerals") &&
                !currentPath.getCurrentSegment().getName().equals("back up from minerals")) && AbstractOpMode.isOpModeActive());
        beginIntaking();
        while ((currentPath.getCurrentSegment().getName().equals("drive to minerals") ||
                currentPath.getCurrentSegment().getName().equals("back up from minerals")) && AbstractOpMode.isOpModeActive());
        finishIntaking();
    }

    public synchronized void beginIntaking() {
        intake.setIntakePower(INTAKE_FORWARD_POWER);
    }

    public synchronized void finishIntaking() {
        intake.setIntakePower(INTAKE_STOP_POWER);
    }

    public synchronized void reverseIntake() {
        intake.setIntakePower(INTAKE_REVERSE_POWER);
    }

    public synchronized void slowReverseIntake() {
        intake.setIntakePower(0.1);
    }
}