package org.firstinspires.ftc.teamcode.robots.ri2d2023;

import com.acmerobotics.dashboard.config.Config;

@Config("IronGiantAutonVariables")
abstract class Task {
    public static int TICKSPERTILE = 290;
    public static int TICKSPER90DEGREES = 50;
    public static int STRAFETICKSPERTILE = 290;
    public static float MAXMOTORSPEED = .7f;
    public static final int NANOTOSECOND = 1000000000;
    public static double TIMEBETWEENTASKS = 1 ;
    abstract boolean run();
}
