package org.firstinspires.ftc.teamcode.utils;

public class ThreadUtils {
    public static final double REST_TIME = 200;

    public static boolean runThread = true;

    public static void rest() {
        rest(200);
    }

    public static void rest(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public static void startThreads() {
        runThread = true;
    }

    public static void stopThreads() {
        runThread = false;
    }

    public static boolean isRunThread() {
        return runThread;
    }
}
