package org.firstinspires.ftc.teamcode.BBcode.UtilClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Time {
    static ElapsedTime timer = null;
    public static boolean Wait(double seconds) {
        if (timer == null) {ElapsedTime timer = new ElapsedTime();}
        if (timer.seconds() >= seconds) {
            timer = null;
            return true;
        }
        return false;
    }
}
