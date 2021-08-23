package org.firstinspires.ftc.teamcode.util;

public class Utils {
    public static final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
