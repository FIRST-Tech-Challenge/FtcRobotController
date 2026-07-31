package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FlywheelShooterConfig {
    public static double IDLE_VELOCITY = 1200;
    public static double SHOOTING_VELOCITY = 2000;
    public static double STALLED_VELOCITY = 0;
    public static double RPM_THRESHOLD = 100;

    public static double kP = 0.0;
    public static double kV = 0.0;
    public static double kS = 0.0;

    // ----SINGLE MOTOR FLYWHEEL CONFIG----
    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // ----DUAL MOTOR FLYWHEEL CONFIG----
    public static DcMotorSimple.Direction LEFT_FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

}
