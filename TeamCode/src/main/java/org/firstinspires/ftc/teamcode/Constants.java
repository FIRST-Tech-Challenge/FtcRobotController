package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import org.firstinspires.ftc.teamcode.AprilTagLocation;

public final class Constants {
    public static final double TICKS_PER_REVOLUTION = 2000;
    public static final double DIAMETER = 1.889764;
    public static final double CIRCUMFERENCE = PI * DIAMETER;
    public static final AprilTagLocation[] APRIL_TAG_LOCATIONS = new AprilTagLocation[] {
            new AprilTagLocation(1, new double[] {0, 0, 0}),
            new AprilTagLocation(2, new double[] {36, 0, 0})
    };
}
