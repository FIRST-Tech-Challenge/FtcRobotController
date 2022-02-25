package org.firstinspires.ftc.teamcode.opmodes.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DelayStorage {
    public static int seconds = 0;

    public static void setSeconds(int seconds) {
        DelayStorage.seconds = seconds;
    }

    public static void addSeconds(int seconds) {
        DelayStorage.seconds += seconds;
    }

    public static void subtractSeconds(int seconds) {
        if (DelayStorage.seconds - seconds < 0) {
            DelayStorage.seconds = 0;
        } else {
            DelayStorage.seconds -= seconds;
        }
    }

    public static void waitForDelay(ElapsedTime timer) {
        while (timer.seconds() < seconds) {
            // waiting...
        }
    }
}
