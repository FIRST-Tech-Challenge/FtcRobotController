package org.firstinspires.ftc.teamcode.robots.catbot;

interface Task {
    int TICKSPERTILE = 700;
    int TICKSPER90DEGREES = 525;
    int STRAFETICKSPERTILE = 1000;
    float MAXMOTORSPEED = .7f;
    boolean run();
}
