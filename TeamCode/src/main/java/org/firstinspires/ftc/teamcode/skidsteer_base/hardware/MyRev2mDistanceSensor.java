package org.firstinspires.ftc.teamcode.skidsteer_base.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;

/**
 * Created by user on 9/30/2018.
 */

public class MyRev2mDistanceSensor {


    private HardwareMap hwMap;
    private com.qualcomm.hardware.rev.Rev2mDistanceSensor sensorTimeOfFlight1;
    private DistanceSensor sensorRange1;
    private String InputString;


    public MyRev2mDistanceSensor( String CfgNameOnPhone ){

        InputString = CfgNameOnPhone;
        // This is an odd way to fix the hardware map issue
        //    where the normal hardware map getter does not work.
        hwMap = AbstractTeleop.getOpModeInstance().hardwareMap;

        sensorRange1 = hwMap.get(DistanceSensor.class, CfgNameOnPhone);
        sensorTimeOfFlight1 = (com.qualcomm.hardware.rev.Rev2mDistanceSensor)sensorRange1;

    }


    public double getDistanceIN() {

        if (InputString == "distance2") {
            return (sensorRange1.getDistance(DistanceUnit.INCH) + 0.5);
        } else {
            return (sensorRange1.getDistance(DistanceUnit.INCH));
        }
    }
}
