package org.firstinspires.ftc.teamcode.framework.userhardware.outputs;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Logger {
    private final File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM);
    private File file;

    private FileOutputStream fOut;
    private OutputStreamWriter myOutWriter;

    public Logger(String fileName) {
        if (fileName == null) {
            file = null;
            return;
        }
        try {
            file = new File(path, "FTC RobotController Phone Log " + fileName + " [" + new SimpleDateFormat("yyyy.MM.dd HH:mm:ss.sss").format(new Date()) + "].txt");
            int n = 0;
            while (file.exists()) {
                file = new File(path, "FTC RobotController Phone Log " + fileName + " [" + new SimpleDateFormat("yyyy.MM.dd HH:mm:ss.sss").format(new Date()) + "](" + n + ").txt");
            }
            RobotLog.i("ABCD Creating New File: " + file.getName());
            file.createNewFile();
            fOut = new FileOutputStream(file);
            myOutWriter = new OutputStreamWriter(fOut);
        } catch (Exception e) {
            Log.e("Exception", "File init failed " + e.toString());
        }
    }

    public void log(String text) {
        if (file == null) return;

        String timeStamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date());

        try {
            myOutWriter.append(timeStamp + " : " + text + (char) Character.LINE_SEPARATOR);
            myOutWriter.flush();
        } catch (Exception e) {
            Log.e("Exception", "File append failed: " + e.toString());
        }
    }

    public void stop() {
        if (file == null) return;

        try {
            myOutWriter.close();
            fOut.close();
        } catch (IOException e) {
            Log.e("Exception", "File close failed " + e.toString());
        }
    }
}
