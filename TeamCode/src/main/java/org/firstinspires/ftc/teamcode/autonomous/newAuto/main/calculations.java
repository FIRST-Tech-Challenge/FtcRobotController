package org.firstinspires.ftc.teamcode.autonomous.newAuto.main;


import java.util.Random;

public class calculations {
    // Measurements for robots width, length, height, and wheels circumference
    public final double WHEEL_CIRC = 3.75 * Math.PI;
    public final double ROBOT_LENGTH = 14;
    public final double ROBOT_WIDTH = 16;
    public final double ROBOT_HEIGHT = 0;//TODO find robot height
    public final double TURNING_RADIUS = Math.sqrt( (Math.pow(ROBOT_LENGTH, 2) )+( (Math.pow(ROBOT_WIDTH, 2) ) ) );
    public final double DISTANCE_TO_BLOCKS = 0;//TODO find distance to blocks
    public final double SMIDGEN = 0.1;
    public final double DISTANCE_FROM_WALL = 0.5;

    public final double tick_per_inch = 0;
    public final double tick_per_360 = 0;

    public final double MANTIS_TICK_PER_INCH = 0;
    public final double LIFT_TICK_PER_INCH = 0;
    public final double HOPPER_TICK_PER_INCH = 0;

    // The normal drive speed
    public final double DRIVE_SPEED = 0.2;

    public boolean yellow = false;

    public Random random = new Random();

}
