package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SCARAController {

    final static double SIDE_TO_SIDE_RANGE = 8 * 25.4;
    final static double MIDLINE = -76;
    final static double CALIBRATION_Y_DISTANCE = 200;

    //shift delivery and pickup forward by 25 cm to account for the new claw attach point
    final static double DELIVER_Y_DISTANCE = 6.5 * 25.4 - 25;

    final static double PICK_UP_Y_DISTANCE = -180; // had -25 (negative closer to front of robot)


    Telemetry telemetry;

    /*
     * Return the angle difference travelling clockwise from startAngle to endAngle
     */
    public double getAngleDifferenceCounterClockwise(double startAngle, double endAngle) {
        if (startAngle <= endAngle) {
            return endAngle - startAngle;
        } else {
            return endAngle - startAngle + 2 * Math.PI;
        }
    }

    /*
     * Return the angle difference travelling clockwise from startAngle to endAngle
     */
    public double getSignedAngleDifferenceCounterClockwise(double startAngle, double endAngle, double threshold) {
        double difference = endAngle - startAngle;
        if (startAngle > endAngle) {
            difference += 2 * Math.PI;
        }
        if (difference > threshold) {
            difference -= 2 * Math.PI;
        }
        return difference;
    }



    private double scaleAngleWithWraparound(double a1, double a2, double scale) {
        double diff = a2 - a1;
        if (Math.abs(diff) < Math.PI) return a1 + diff * scale; // no wraparound
        if (diff < 0) diff += 2 * Math.PI;
        else diff -= 2 * Math.PI;
        double result = a1 + diff * scale;
        if (result >= Math.PI) result -= 2 * Math.PI;
        if (result < -Math.PI) result += 2 * Math.PI;
        return result;
    }

    public class Coordinates {
        double x, y;

        public Coordinates(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Coordinates(Coordinates other) {
            this(other.x, other.y);
        }

        public boolean copy(Coordinates other) {
            this.x = other.x;
            this.y = other.y;
            return true;
        }

    }
}