package org.firstinspires.ftc.teamcode.subsystems;

public class Util {
    public static double cosInDegrees(double angle) {
        return Math.cos(angle * Math.PI / 180);
    }

    public static double sinInDegrees(double angle) {
        return Math.sin(angle * Math.PI / 180);
    }

    public static double aCosInDegrees(double ratio) {
        return Math.acos(ratio) / Math.PI * 180;
}
