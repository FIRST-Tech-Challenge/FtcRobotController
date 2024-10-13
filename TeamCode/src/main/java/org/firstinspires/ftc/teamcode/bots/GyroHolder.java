package org.firstinspires.ftc.teamcode.bots;

public class GyroHolder {
    private static double lastHeading = 0;

    public static void setHeading(double heading) {
        lastHeading = heading;
    }

    public static double getHeading() {
        return lastHeading;
    }
}
