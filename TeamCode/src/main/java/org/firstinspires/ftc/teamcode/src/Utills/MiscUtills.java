package org.firstinspires.ftc.teamcode.src.Utills;

import java.io.PrintWriter;
import java.io.StringWriter;

public class MiscUtills {
    /**
     * @param num this is a number
     * @return It returns the given number within a bound of -1 to 1
     * this method is often used to adjust variables used for drive control
     */
    public static double boundNumber(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

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
     * @param e this is any exception
     * @return It returns a string of the stack trace from the given exception
     * this method is helpful for debugging problems within opmodes by outputing this string in telemetry
     */
    public static String getStackTraceAsString(Exception e) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        return sw.toString();
    }

    /**
     * @param m this is the slope of the line
     * @param b this is the y intercept of the line
     * @param x this is the x value
     * @return It returns the y value
     */
    public double linearEquation(double m, double b, double x) {
        double y = (m * x) + b;
        return y;
    }


}
