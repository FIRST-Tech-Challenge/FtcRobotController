package org.firstinspires.ftc.teamcode;

/*this class contains methods for doing math
this should reduce clutter in SubRobot
*/
public class RobotMath {

    //finds absolute difference between to values
    static boolean isAbsDiffWithinRange(double num1, double num2, double allowedError) {
        return absDiff(num1, num2) < allowedError;
    }
    static double absDiff(double num1, double num2) {
        return Math.abs(num1 - num2);
    }
    //converts gyro angle into unit circle
    static double toUnitCircle(double angle) {
        angle *= -1;
        angle += 90;

        if (angle < 0) {
            angle += 360;
        }

        return angle;
    }
    //returns angle (degrees) on unit circle. All angles are positive
    static double atanUnitCircle(double o, double a) {
        double angle = Math.toDegrees(Math.atan2(o, a));
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }
    //turns unit cirlce anlge to 0 at top, + left, - right
    static double unitCircleTo0AtTop(double angle) {
        angle -= 90;
        angle = wrapAround(angle);
        return angle;
    }

    //if an angle is greater than 180, it will wrap the angle around by subtracting 360
    static double wrapAround(double angle) {
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    static double round(double num, double numOfDecimals) {
        return Math.round(num * Math.pow(10, numOfDecimals)) / Math.pow(10, numOfDecimals);
    }
    static double round(double num) {
        return round(num, 2);
    }

    static double deadZone(double num, double deadZone) {
        if (Math.abs(num) < deadZone) {
            return 0;
        } else {
            return num;
        }

    }

    //this gives the angle that is the opposite direction
    public static double flipAngle(double angle) {
        if (angle >= 180) {
            return angle - 180;
        }
        return angle + 180;
    }
    public static double maxAndMin(double value, double max, double min) {
        if (value > max) {
            return max;
        } else if (value < min) {
            return min;
        }
        return value;
    }
}