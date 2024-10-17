package org.firstinspires.ftc.teamcode.resourses;

public class Utlities {
    // used to keep angles within a range of 360 even when turning on for a while
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
