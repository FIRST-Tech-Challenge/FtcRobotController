package org.firstinspires.ftc.teamcode.resourses;

public class Utlities {
    // used to keep angles within a range of 360 even when turning on for a while
    public static double wrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
