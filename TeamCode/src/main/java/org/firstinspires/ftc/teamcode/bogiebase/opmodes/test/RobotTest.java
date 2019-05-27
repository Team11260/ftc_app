package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.Speech;

@TeleOp(name = "Robot Test", group = "Test")
//@Disabled

public class RobotTest extends AbstractAutonNew {

    Robot robot;
    Speech speech;

    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {

        RobotState.currentMatchState = RobotState.MatchState.TELEOP;

        robot = new Robot();
        speech = new Speech(hardwareMap);
    }

    @Override
    public void Run() {

        telemetry.setDefaultLogMode(DoubleTelemetry.LogMode.INFO);

        telemetry.addDataPhone("To run test follow instructions");
        telemetry.addDataPhone("");
        telemetry.addDataPhone("Press (a) to continue");
        telemetry.update();

        speech.speak("To run test follow instructions, press a to continue");

        while (speech.isSpeaking() && !gamepad1.a);
        speech.stop();

        while (!gamepad1.a);
        while (gamepad1.a);

        diagnostics();

        testDrive();

        testMineralLift();

        testRobotLift();
    }

    public void diagnostics() {

        telemetry.addDataPhone("Voltage: " + robot.getVoltage());
        telemetry.addDataPhone("Current: " + robot.getCurrent());

        if(robot.getVoltage() < 13.5) {
            speech.speak("Low battery");
            while (speech.isSpeaking());
        }

        if(robot.getCurrent() < 0.05) {
            speech.speak("High current");
            while (speech.isSpeaking());
        }

        telemetry.addDataPhone("");
        telemetry.addDataPhone("Press (a) to continue");
        telemetry.update();

        while (!gamepad1.a);
        while (gamepad1.a);
    }

    public void testDrive() {
        telemetry.addDataPhone("Drive train test:");
        telemetry.addDataPhone("Press (a) to start, press (b) to skip");
        telemetry.update();

        speech.speak("Drive train test, press a to start, press b to skip");

        while (speech.isSpeaking() && !gamepad1.a && !gamepad1.b);
        speech.stop();

        while (true) {
            if(gamepad1.a) {
                while (gamepad1.a);

                String testText = "Test successful";

                robot.setDrivePowerNoEncoder(1, 1);

                delay(5000);

                robot.setDrivePowerNoEncoder(0, 0);

                delay(2000);

                telemetry.addDataPhone("Drive left position for segment 1: " + robot.getLeftDrivePosition());
                telemetry.addDataPhone("Drive right position for segment 1: " + robot.getRightDrivePosition());

                if(robot.getLeftDrivePosition() < 10000 || robot.getLeftDrivePosition() > 20000 || robot.getRightDrivePosition() < 10000 || robot.getRightDrivePosition() > 20000 || Math.abs(robot.getLeftDrivePosition() - robot.getRightDrivePosition()) > 5000) testText = "Test failed on segment 1";

                robot.resetDrivePosition();

                robot.setDrivePowerNoEncoder(-1, -1);

                delay(5000);

                robot.setDrivePowerNoEncoder(0, 0);

                delay(2000);

                telemetry.addDataPhone("");
                telemetry.addDataPhone("Drive left position for segment 2: " + robot.getLeftDrivePosition());
                telemetry.addDataPhone("Drive right position for segment 2: " + robot.getRightDrivePosition());

                if(robot.getLeftDrivePosition() > -10000 || robot.getLeftDrivePosition() < -20000 || robot.getRightDrivePosition() > -10000 || robot.getRightDrivePosition() < -20000 || Math.abs(robot.getLeftDrivePosition() - robot.getRightDrivePosition()) > 5000) testText = "Test failed on segment 2";

                robot.setDrivePowerNoEncoder(1, 1);
                delay(500);
                robot.setDrivePowerNoEncoder(-1, -1);
                delay(500);
                robot.setDrivePowerNoEncoder(1, 1);
                delay(500);
                robot.setDrivePowerNoEncoder(-1, -1);
                delay(500);
                robot.setDrivePowerNoEncoder(1, 1);
                delay(500);
                robot.setDrivePowerNoEncoder(-1, -1);
                delay(500);
                robot.setDrivePowerNoEncoder(0, 0);

                robot.resetDrivePosition();

                telemetry.addDataPhone("");
                telemetry.addDataPhone(testText);
                telemetry.addDataPhone("");
                telemetry.addDataPhone("Press (a) to continue");
                telemetry.update();

                speech.speak("Drive train " + testText + " press a to continue");

                while (speech.isSpeaking() && !gamepad1.a);
                speech.stop();

                while (!gamepad1.a);
                while (gamepad1.a);

                break;
            }

            if(gamepad1.b) {
                while (gamepad1.b);
                break;
            }
        }
    }

    public void testMineralLift() {
        telemetry.addDataPhone("Mineral lift test:");
        telemetry.addDataPhone("Press (a) to start, press (b) to skip");
        telemetry.update();

        speech.speak("Mineral lift test, press a to start, press b to skip");

        while (speech.isSpeaking() && !gamepad1.a && !gamepad1.b);
        speech.stop();

        while (true) {

            if(gamepad1.a) {

                String testText = "Test successful";

                while (gamepad1.a);

                robot.moveMineralLiftToDumpPosition();

                telemetry.addDataPhone("Mineral lift position for raise: " + robot.getMineralLiftPosition());

                if(robot.getMineralLiftPosition() < 900 || robot.getMineralLiftPosition() > 1100) testText = "Test failed on raise";

                robot.moveMineralLiftToCollectPosition();

                telemetry.addDataPhone("");
                telemetry.addDataPhone("Mineral lift position for lower: " + robot.getMineralLiftPosition());

                if(robot.getMineralLiftPosition() > 20 || robot.getMineralLiftPosition() < -20) testText = "Test failed on lower";

                telemetry.addDataPhone("");
                telemetry.addDataPhone(testText);
                telemetry.addDataPhone("");
                telemetry.addDataPhone("Press (a) to continue");
                telemetry.update();

                speech.speak("Mineral lift " + testText + " press a to continue");

                while (speech.isSpeaking() && !gamepad1.a);
                speech.stop();

                while (!gamepad1.a);
                while (gamepad1.a);

                break;
            }

            if(gamepad1.b) {
                while (gamepad1.b);
                break;
            }
        }
    }

    public void testRobotLift() {
        telemetry.addDataPhone("Robot lift test:");
        telemetry.addDataPhone("Press (a) to start, press (b) to skip");
        telemetry.update();

        speech.speak("Robot lift test, press a to start, press b to skip");

        while (speech.isSpeaking() && !gamepad1.a && !gamepad1.b);
        speech.stop();

        while (true) {

            if(gamepad1.a) {

                String testText = "Test successful";

                while (gamepad1.a);

                robot.robotLiftUp();

                delay(1000);

                robot.robotLiftStop();

                telemetry.addDataPhone("Robot lift position for lower: " + robot.getMineralLiftPosition());

                robot.robotLiftDown();

                delay(1400);

                robot.robotLiftStop();

                telemetry.addDataPhone("");
                telemetry.addDataPhone("Robot lift position for raise: " + robot.getMineralLiftPosition());

                telemetry.addDataPhone("");
                telemetry.addDataPhone(testText);
                telemetry.addDataPhone("");
                telemetry.addDataPhone("Press (a) to continue");
                telemetry.update();

                speech.speak("Robot lift " + testText + " press a to continue");

                while (speech.isSpeaking() && !gamepad1.a);
                speech.stop();

                while (!gamepad1.a);
                while (gamepad1.a);

                break;
            }

            if(gamepad1.b) {
                while (gamepad1.b);
                break;
            }
        }
    }
}
