package org.firstinspires.ftc.teamcode.robots.catbot;

import com.acmerobotics.dashboard.config.Config;

@Config("IronGiantAutonVariables")
abstract class Task {
    public static int TICKSPERTILE = 350;
    public static int TICKSPER90DEGREES = 90;
    public static int STRAFETICKSPERTILE = 350;
    public static float MAXMOTORSPEED = .7f;
    public static final int NANOTOSECOND = 1000000000;
    public static double TIMEBETWEENTASKS = .5;
    abstract boolean run();
}
