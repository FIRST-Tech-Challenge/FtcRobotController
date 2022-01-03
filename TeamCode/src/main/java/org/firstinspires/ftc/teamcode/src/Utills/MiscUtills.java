package org.firstinspires.ftc.teamcode.src.Utills;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * A class full of miscellaneous utilities
 */
public class MiscUtills {
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
    public static String getStackTraceAsString(Exception e) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        return sw.toString();
    }

    /**
     * It is used to calculate a point on a line
     * @param m this is the slope of the line
     * @param b this is the y intercept of the line
     * @param x this is the x value
     * @return It returns the y value
     */
    public double linearEquation(double m, double b, double x) {
        return (m * x) + b;
    }


}
