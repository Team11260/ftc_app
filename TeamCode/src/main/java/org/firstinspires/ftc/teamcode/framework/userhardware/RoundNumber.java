package org.firstinspires.ftc.teamcode.framework.userhardware;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class RoundNumber {

    double value;
    int places;

    public RoundNumber() {
        value = 0.0;
        places = 1;
    }

    public double roundDouble(double value, int places) {
        if (places < 0) {
            places = 0;
        }
        BigDecimal bd = new BigDecimal(Double.toString(value));
        // why is there a bd = ?
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
