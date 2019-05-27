package org.upacreekrobotics.dashboard;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DashboardGamepad extends Gamepad {

    public DashboardGamepad() {

    }

    public void update(String string) {
        String[] parts = string.split("<:-:>");

        for (int i = 0; i < parts.length; i++) {
            String part = parts[i];

            String key = part.split(":")[0];
            String value = part.split(":")[1];

            switch (key) {
                case "a":
                    a = Boolean.valueOf(value);
                    break;
                case "b":
                    b = Boolean.valueOf(value);
                    break;
                case "x":
                    x = Boolean.valueOf(value);
                    break;
                case "y":
                    y = Boolean.valueOf(value);
                    break;
                case "dpad_down":
                    dpad_down = Boolean.valueOf(value);
                    break;
                case "dpad_right":
                    dpad_right = Boolean.valueOf(value);
                    break;
                case "dpad_left":
                    dpad_left = Boolean.valueOf(value);
                    break;
                case "dpad_up":
                    dpad_up = Boolean.valueOf(value);
                    break;
                case "left_stick_button":
                    left_stick_button = Boolean.valueOf(value);
                    break;
                case "right_stick_button":
                    right_stick_button = Boolean.valueOf(value);
                    break;
                case "left_bumper":
                    left_bumper = Boolean.valueOf(value);
                    break;
                case "right_bumper":
                    right_bumper = Boolean.valueOf(value);
                    break;
                case "back":
                    back = Boolean.valueOf(value);
                    break;
                case "start":
                    start = Boolean.valueOf(value);
                    break;
                case "mode":
                    guide = Boolean.valueOf(value);
                    break;
                case "left_stick_x":
                    left_stick_x = Float.valueOf(value);
                    break;
                case "left_stick_y":
                    left_stick_y = Float.valueOf(value);
                    break;
                case "right_stick_x":
                    right_stick_x = Float.valueOf(value);
                    break;
                case "right_stick_y":
                    right_stick_y = Float.valueOf(value);
                    break;
                case "left_trigger":
                    left_trigger = Float.valueOf(value);
                    break;
                case "right_trigger":
                    right_trigger = Float.valueOf(value);
                    break;
            }
        }
    }

    public void update(android.view.MotionEvent event) {

    }

    public void update(android.view.KeyEvent event) {

    }

    public void copy(Gamepad gamepad) throws RobotCoreException {

    }

    public void reset() {

    }

    public void fromByteArray(byte[] byteArray) throws RobotCoreException {

    }
}
