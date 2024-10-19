package org.firstinspires.ftc.teamcode.resourses;

public class Utlities {
    // used to keep angles within a range of 360 even when turning on for a while
    // moved wrap function in here so we can access it more easily
    public static double wrap(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}
