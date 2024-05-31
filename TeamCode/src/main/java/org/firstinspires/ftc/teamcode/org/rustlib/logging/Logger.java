package org.firstinspires.ftc.teamcode.org.rustlib.logging;

import android.util.Pair;

import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;

public class Logger {
    private static Pair<Double, Double> batteryVoltage;
    private static Pair<Double, Pose2d> botPose;

    public static void log(String data) {

    }

    public static void log(Object o) {
        log(o.toString());
    }

}
