package org.firstinspires.ftc.teamcode.src.Utills;

import java.io.PrintWriter;
import java.io.StringWriter;

public class MiscUtills {
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

    public static String getStackTraceAsString(Exception e) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        return sw.toString();
    }

    public double linearEquation(double m, double b, double x) {
        double y = (m * x) + b;
        return y;
    }


}
