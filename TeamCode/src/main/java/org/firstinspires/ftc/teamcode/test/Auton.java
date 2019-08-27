package org.firstinspires.ftc.teamcode.test;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.vuforia.SampleVuforia;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.Speech;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;

@Autonomous(name="Test_Auto_New", group="New")
//@Disabled

public class Auton extends AbstractAuton {

    Speech speech;
    SampleVuforia vuforia;

    @Override
    public void Init() {
        telemetry.addData("Init");
        telemetry.update();
        speech = new Speech(hardwareMap);
        vuforia = new SampleVuforia();
    }

    @Override
    public void InitLoop(){

    }

    @Override
    public void Run() {
        speech.speak("Program Running");
        telemetry.addData("Run");
        telemetry.update();

        // Get the directory for the user's public pictures directory.
        final File path =
                Environment.getExternalStoragePublicDirectory
                        (
                                //Environment.DIRECTORY_PICTURES
                                Environment.DIRECTORY_DCIM
                        );

        final File file = new File(path, "mylog.txt");

        try{
            file.createNewFile();
            FileOutputStream fOut = new FileOutputStream(file);
            OutputStreamWriter myOutWriter = new OutputStreamWriter(fOut);

            myOutWriter.append("test log");
            myOutWriter.close();
        }
        catch (IOException e)
        {
            Log.e("Exception", "File write failed: " + e.toString());
        }


    }

    @Override
    public void Stop() {
        telemetry.addData("Stop");
        telemetry.update();
    }
}