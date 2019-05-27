package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.tensorflow.TensorFlowImpl;

import static org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition.UNKNOWN;

@Autonomous(name = "TensorFlowImpl Test", group = "New")
@Disabled

public class TensorFlowTest extends AbstractAutonNew {

    private TensorFlowImpl tensorFlow;
    private SamplePosition lastPosition = SamplePosition.UNKNOWN;

    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {
        tensorFlow = new TensorFlowImpl(TensorFlowImpl.CameraOrientation.VERTICAL, "Webcam 1", false);
        //tensorFlow = new TensorFlowImpl(TensorFlowImpl.CameraOrientation.HORIZONTAL, false);
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
