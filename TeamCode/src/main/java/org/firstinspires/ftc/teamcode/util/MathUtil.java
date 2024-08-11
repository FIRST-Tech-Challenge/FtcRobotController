package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class MathUtil {
    public static double countsToDeg(int counts, double maxCounts) {
        return ((counts * 360) / maxCounts);
    }

    public static double degToCounts(double deg, double maxCounts) {
        return ((deg * maxCounts) / 360.0);
    }

    public static double encoderTicksToMeter(double ticks) {
        return ticks * RobotConfig.DriveTrain.METERS_PER_TICK;
    }
}
