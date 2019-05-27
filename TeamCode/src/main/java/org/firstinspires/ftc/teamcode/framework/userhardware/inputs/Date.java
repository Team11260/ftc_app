package org.firstinspires.ftc.teamcode.framework.userhardware.inputs;

import java.text.SimpleDateFormat;

public class Date {
    private SimpleDateFormat dtf;

    public Date() {
        this("dd/MM/yyyy HH:mm:ss");
    }

    public Date(String format) {
        dtf = new SimpleDateFormat(format);
    }

    public String getDate() {
        return dtf.format(new java.util.Date());
    }
}
