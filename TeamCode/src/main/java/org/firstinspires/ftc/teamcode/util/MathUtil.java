package org.firstinspires.ftc.teamcode.util;

public class MathUtil {
    public static double countsToDeg(int counts, double maxCounts) {
        return ((counts * 360) / maxCounts);
    }

    public static double degToCounts(double deg, double maxCounts) {
        return ((deg * maxCounts) / 360.0);
    }
}
