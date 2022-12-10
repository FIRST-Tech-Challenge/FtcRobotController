package org.firstinspires.ftc.teamcode.robots.catbot;

interface Task {
    int TICKSPERTILE = 2500;
    int TICKSPER90DEGREES = 1000;
    int STRAFETICKSPERTILE = 2800;
    boolean run();
}
