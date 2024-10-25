package org.firstinspires.ftc.teamcode.autonomous.newAuto.classes;


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

    public final double MANTIS_LENGTH_INCH = 30;
    public final double LIFT_LENGTH_INCH = 30;
    public final double HOPPER_LENGTH_INCH = 30;

    public final double WHEEL_MOTOR_TICK = 1538;
    public final double WHEEL_TICK_PER_INCH = WHEEL_MOTOR_TICK / WHEEL_CIRC;
    // TODO experiment with this number until it rotates a perfect 360
    public final double WHEEL_TICK_PER_360 = 1267.69;
    public final double WHEEL_TICK_PER_DEGREE = WHEEL_TICK_PER_360 / 360;

    public final double MANTIS_MOTOR_TICK = 50;
    public final double LIFT_MOTOR_TICK = 50;
    public final double HOPPER_MOTOR_TICK = 50;

    public final double MANTIS_TICK_PER_EXTEND = MANTIS_MOTOR_TICK * 5;
    public final double LIFT_TICK_PER_EXTEND = LIFT_MOTOR_TICK * 5;
    public final double HOPPER_TICK_PER_EXTEND = HOPPER_MOTOR_TICK * 5;

    public final double MANTIS_TICK_PER_INCH = MANTIS_TICK_PER_EXTEND / MANTIS_LENGTH_INCH;
    public final double LIFT_TICK_PER_INCH = LIFT_TICK_PER_EXTEND / LIFT_LENGTH_INCH;
    public final double HOPPER_TICK_PER_INCH = HOPPER_TICK_PER_EXTEND / HOPPER_LENGTH_INCH;

    // The normal drive speed
    public final double DRIVE_SPEED = 0.2;

    public boolean yellow = false;

    public Random random = new Random();

}
