package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseHolder {
    public static Pose2d currentDrivePose = new Pose2d();
    public static int slideHeight;

    public static int START_TICKS = (int)(300 / 1.5);
    public static int INTAKE_TICKS = 0;
    public static int LOW_TICKS = (int)(2063 / 1.5);
    public static int MID_TICKS = (int)(3500 / 1.5);
    public static int HIGH_TICKS = (int)(4900 / 1.5);
}
