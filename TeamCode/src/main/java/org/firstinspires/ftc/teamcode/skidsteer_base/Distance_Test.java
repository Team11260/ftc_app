package org.firstinspires.ftc.teamcode.skidsteer_base;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;

@TeleOp(name="SkidSteer_DistanceTest", group="Utility")
//@Disabled

public class Distance_Test extends AbstractTeleop {
    private Rev2mDistanceSensor sensorTimeOfFlight;
    private DistanceSensor sensorRange;

    @Override
    public void Init() {
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
    }

    @Override
    public void Start(){

    }

    @Override
    public void Loop() {
        telemetry.addData(String.format("Sensor Range %2.2f", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.update();
    }
}
