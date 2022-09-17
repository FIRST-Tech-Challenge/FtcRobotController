package org.firstinspires.ftc.forteaching.util;

public class SleepHelper {
    public static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
        }
    }
}
