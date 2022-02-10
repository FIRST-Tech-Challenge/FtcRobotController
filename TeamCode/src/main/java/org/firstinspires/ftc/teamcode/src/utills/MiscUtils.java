package org.firstinspires.ftc.teamcode.src.utills;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.concurrent.locks.Lock;

/**
 * A class full of miscellaneous utilities
 */
public class MiscUtils {
    /**
     * This constrains the input number between 1 and -1 inclusive
     *
     * @param num the number to constrain
     * @return the number after it has been constrained
     */
    public static double boundNumber(double num) {
        if (num > 1) {
            num = 1;
        }
        if (num < -1) {
            num = -1;
        }
        return num;
    }

    /**
     * This constrains the input number between 1 and -1 inclusive
     *
     * @param num the number to constrain
     * @return the number after it has been constrained
     */
    public static float boundNumber(float num) {
        if (num > 1.0) {
            return 1.0F;
        }
        if (num < -1.0) {
            return -1.0F;
        }
        return num;
    }

    /**
     * It takes a exception and returns the string stack trace this method is helpful for debugging problems within OpModes by sending a string to telemetry
     *
     * @param e this is any exception
     * @return It returns a string of the stack trace from the given exception
     */
    public static String getStackTraceAsString(Throwable e) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        return sw.toString();
    }

    /**
     * It is used to calculate a point on a line
     *
     * @param m this is the slope of the line
     * @param b this is the y intercept of the line
     * @param x this is the x value
     * @return It returns the y value
     */
    public static double linearEquation(double m, double b, double x) {
        return (m * x) + b;
    }

    /**
     * Maps value that is in range [a, b] into the range [c, d]
     *
     * @param value The value to be mapped
     * @param a     The lower bound of the native range
     * @param b     The upper bound of the native range
     * @param c     The lower bound of the new range
     * @param d     The upper bound of the new range
     * @return The new value mapped to the new range
     */
    public static double map(double value, double a, double b, double c, double d) {
        return ((value - a) * ((d - c) / (b - a))) + c;
    }

    /**
     * Determines the distance between two points
     *
     * @param x1 the x-value of the first point
     * @param y1 the y-value of the first point
     * @param x2 the x-value of the second point
     * @param y2 the y-value of the second point
     * @return The distance between two points
     */
    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    }

    /**
     * This is used to get the angle between two points
     *
     * @param rx       The robot x position
     * @param ry       Robot Y Position
     * @param x        X Position to go to
     * @param y        Y position to go to
     * @param robotRot The orientation of the robot in degrees
     * @return The heading the point is from the robot in degrees
     */
    public static double getAngle(double rx, double ry, double x, double y, double robotRot) {
        double angle;
        x = x - rx;
        y = y - ry;
        angle = Math.toDegrees(Math.atan2(x, y));
        return ((angle - robotRot) % 360);
    }

    public static String getRelativeClassName(Object o) {
        String cName = o.getClass().toString();
        String[] cNameSplit = cName.split("\\.");
        cName = cNameSplit[cNameSplit.length - 1];
        return cName;
    }

    /**
     * Acquires all locks passed to it. Blocks until acquires all locks. Will not cause deadlock
     *
     * @param locks A array of lock objects
     * @throws InterruptedException If the thread is interrupted during execution
     */
    public static void getLocks(Lock[] locks) throws InterruptedException {
        boolean[] gotLocks = new boolean[locks.length];

        boolean gotAllLocks;
        while (true) {
            try {
                //Tries to lock each lock
                for (int i = 0; i < locks.length; i++) {
                    gotLocks[i] = locks[i].tryLock();
                }
            } finally {
                //goes through the array of returned booleans. If one of them is false, we do not have all the locks
                //If all of them are true, we do and can return
                gotAllLocks = true;
                for (boolean gotLock : gotLocks) {
                    if (!gotLock) {
                        gotAllLocks = false;
                        break;
                    }
                }
                //If we have all the locks, we can return
                if (!gotAllLocks) {
                    for (int i = 0; i < gotLocks.length; i++) {
                        if (gotLocks[i]) {
                            locks[i].unlock();
                        }
                    }
                }

            }
            if (gotAllLocks) {
                return;
            }
            Thread.sleep(20);

        }
    }


}
