package org.firstinspires.ftc.teamcode.skidsteer_base;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.Robot;

import java.util.Iterator;
import java.util.LinkedList;

@Autonomous(name="WallCrawler", group="AAA")
//@Disabled

public class WallCrawler extends AbstractAuton {

    private Rev2mDistanceSensor distanceSensor1, distanceSensor2;

    private Robot robot;

    private MovingAvgQueue queue1 = new MovingAvgQueue(10);
    private MovingAvgQueue queue2 = new MovingAvgQueue(10);

    @Override
    public void Init() {
        distanceSensor1 = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "distance1");
        distanceSensor2 = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "distance2");

        robot = new Robot();
    }

    @Override
    public void Run() {
        while (true) {
            queue1.shift(distanceSensor1.getDistance(DistanceUnit.INCH));
            queue2.shift(distanceSensor2.getDistance(DistanceUnit.INCH));
            robot.setDriveY(0.8);

            //double x = queue2.avg() - queue1.avg();
            double x = distanceSensor2.getDistance(DistanceUnit.INCH) - distanceSensor1.getDistance(DistanceUnit.INCH);
            robot.setDriveZ(x/3);

            robot.updateDrive();

            telemetry.addDataPhone("x " + x);
            telemetry.update();
        }
    }

    class MovingAvgQueue extends LinkedList<Double> {
        private int max;
        public MovingAvgQueue(int size) {
            this.max = size;
        }
        public Double avg() {
            if (this.size() == 0 ) {
                return 0.0;
            }
            // Range over the list and calculate the average
            Iterator<Double> it = this.iterator();
            double sum = 0.0;
            while (it.hasNext()) {
                sum += it.next();
            }
            return sum / this.size();
        }

        // poll one from the front, add one to the back.
        public void shift(Double d) {
            if(d > 36.0) {
                // ignore distances greater than 36 inches
                return;
            }
            if (this.size() >= this.max) this.poll();
            this.add(d);
        }
    }
}
