package org.firstinspires.ftc.teamcode.robots.catbot;

import com.acmerobotics.dashboard.config.Config;

@Config("IronGiantAutonVariables")
abstract class Task {
    public static int TICKSPERTILE = 700;
    public static int TICKSPER90DEGREES = 475;
    public static int STRAFETICKSPERTILE = 1000;
    public static float MAXMOTORSPEED = .7f;
    public static final int NANOTOSECOND = 1000000000;
    abstract boolean run();
}
