package org.firstinspires.ftc.teamcode.CV;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CVSettings {
    public static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    public static final double fx = 578.272;
    public static final double fy = 578.272;
    public static final double cx = 402.145;
    public static final double cy = 221.506;

    // UNITS ARE METERS
    public static final double tagsize = 0.166;

    // apriltag tag ID 1,2,3 from the 36h11 family, these are the tags we are interested in
    public static final int LEFT_TAG = 1;
    public static final int MIDDLE_TAG = 2;
    public static final int RIGHT_TAG = 3;
}
