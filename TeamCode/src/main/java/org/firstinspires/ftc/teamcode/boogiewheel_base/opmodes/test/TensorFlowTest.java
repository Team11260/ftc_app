package org.firstinspires.ftc.teamcode.boogiewheel_base.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.userHardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.vision.SamplePosition;
import org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.vision.TensorFlow;
import org.firstinspires.ftc.teamcode.framework.util.State;

import static org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.vision.SamplePosition.UNKNOWN;

@Autonomous(name = "TensorFlow Test", group = "New")
//@Disabled

public class TensorFlowTest extends AbstractAutonNew {

    private TensorFlow tensorFlow;
    private SamplePosition lastPosition = SamplePosition.UNKNOWN;

    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {
        tensorFlow = new TensorFlow(TensorFlow.CameraOrientation.VERTICAL, "Webcam 1", false);
        //tensorFlow = new TensorFlow(TensorFlow.CameraOrientation.HORIZONTAL, false);
    }

    @Override
    public void InitLoop(int loop) {
        if (loop % 5 == 0) {
            tensorFlow.restart();
        }

        SamplePosition currentPosition = tensorFlow.getSamplePosition();

        if (currentPosition != RobotState.currentSamplePosition && currentPosition != UNKNOWN) {
            RobotState.currentSamplePosition = currentPosition;
        }

        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Current Sample Position: " + currentPosition.toString());
        telemetry.update();
    }

    @Override
    public void Run() {

    }

    @Override
    public void Stop() {
        tensorFlow.stop();
    }
}
