package org.firstinspires.ftc.teamcode.opmodes.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DelayStorage {
    public static double seconds = 0;

    public static void setSeconds(double seconds) {
        DelayStorage.seconds = seconds;
    }

    public static void addSeconds(double seconds) {
        DelayStorage.seconds += seconds;
    }

    public static void subtractSeconds(double seconds) {
        if (DelayStorage.seconds - seconds < 0) {
            DelayStorage.seconds = 0;
        } else {
            DelayStorage.seconds -= seconds;
        }
    }

    /**
     * Waits for the delay to be elapsed. Assumes you reset the timer.
     */
    public static void waitForDelay(@NonNull ElapsedTime timer) {
        while (timer.seconds() < seconds) {
            // waiting...
        }
    }
}
