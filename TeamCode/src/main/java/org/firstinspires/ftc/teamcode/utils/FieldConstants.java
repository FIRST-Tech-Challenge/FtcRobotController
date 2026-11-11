package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public class FieldConstants {
    private FieldConstants() {};

    public enum Alliance {BLUE, RED};
    public static Vector2D BLUE_HIVE_A = Vector2D.cartesian(83.75, 61.75); // dx -0.52, dy +3.51
    public static Vector2D BLUE_HIVE_B = Vector2D.cartesian(83.75, 77.75); // dx -0.52, dy -2.10
    public static Vector2D RED_HIVE_A  = Vector2D.cartesian(57.75, 61.75); // dx -0.93, dy +2.06
    public static Vector2D RED_HIVE_B  = Vector2D.cartesian(57.75, 77.75); // dx -0.20, dy -7.54
    public static double HIVE_SWITCH_MARGIN_IN = 6.0;

    public static Vector2D hive(Alliance alliance, boolean hiveA) {
        if (alliance == Alliance.BLUE){
            return hiveA ? BLUE_HIVE_A : BLUE_HIVE_B;
        }
        return hiveA ? RED_HIVE_A : RED_HIVE_B;
    }

    public static double BLUE_DRIVER_FORWARD = Math.toRadians(180);
    public static double RED_DRIVER_FORWARD  = Math.toRadians(0);
    public static double driverForward(Alliance alliance) {
        return alliance == Alliance.BLUE ? BLUE_DRIVER_FORWARD : RED_DRIVER_FORWARD;
    }

    public static Vector2D closestHive(Alliance alliance, double x, double y) {
        Vector2D a = hive(alliance, true);
        Vector2D b = hive(alliance, false);
        return Math.hypot(a.x() - x, a.y() - y) <= Math.hypot(b.x() - x, b.y() - y) ? a : b;
    }

    public static boolean aimHiveA(Alliance alliance, Pose p, Boolean prevA) {
        Vector2D a = hive(alliance, true);
        Vector2D b = hive(alliance, false);
        double dA = Math.hypot(a.x() - p.x(), a.y() - p.y());
        double dB = Math.hypot(b.x() - p.x(), b.y() - p.y());
        if (prevA == null) return dA <= dB;
        return prevA ? !(dB < dA - HIVE_SWITCH_MARGIN_IN) : (dA < dB - HIVE_SWITCH_MARGIN_IN);
    }

    public static Pose endAutoPose = PoseFactory.degrees().of(15.4861, 90.8797, 0);
    public static final Pose RELOCALIZE_POSE = PoseFactory.degrees().of(7.75, 8.21, 90);
}
