package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CenterStageRobot.CenterStageRobot;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

@TeleOp(name = "Romania Bucharest 2024 TeleOP", group = "Final TeleOPs")
public class CenterStageTeleOp extends CommandOpMode {
    private CenterStageRobot robot;

    private DriveConstants RobotConstants;

    public static boolean frontLeftInverted = true;
    public static boolean frontRightInverted = true;
    public static boolean rearRightInverted = true;
    public static boolean rearLeftInverted = true;

    public static double WHEEL_RADIUS = 1.8898; // inch
    public static double GEAR_RATIO = 3.25; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 10.433; // in

    public static double MAX_VEL = 90.0;
    public static double MAX_ACCEL = 90.0;
    public static double MAX_ANG_VEL = Math.toRadians(360);
    public static double MAX_ANG_ACCEL = Math.toRadians(360);

    public static double frontLeftKS = 0;
    public static double frontLeftKV = 1;
    public static double frontLeftKA = 0;
    public static double frontRightKS = 0;
    public static double frontRightKV = 1;
    public static double frontRightKA = 0;
    public static double rearLeftKS = 0;
    public static double rearLeftKV = 1;
    public static double rearLeftKA = 0;
    public static double rearRightKS = 0;
    public static double rearRightKV = 1;
    public static double rearRightKA = 0;

    public static double VELO_KP = 0;
    public static double VELO_KI  = 0;
    public static double VELO_KD = 0;
    public static double minIntegralBound = -400;
    public static double maxIntegralBound = -400;

    public static double TICKS_PER_REV = 145.6;
    public static double MAX_RPM = 1150;

    public static double DEFAULT_SPEED_PERC = 0.75;
    public static double SLOW_SPEED_PERC = 0.3;
    public static double FAST_SPEED_PERC = 1;

    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        RobotConstants = new DriveConstants();

        RobotConstants.frontLeftInverted = frontLeftInverted;
        RobotConstants.frontRightInverted = frontRightInverted;
        RobotConstants.rearRightInverted = rearRightInverted;
        RobotConstants.rearLeftInverted = rearLeftInverted;

        RobotConstants.WHEEL_RADIUS = WHEEL_RADIUS; // inch
        RobotConstants.GEAR_RATIO = GEAR_RATIO; // output (wheel) speed / input (motor) speed
        RobotConstants.TRACK_WIDTH = TRACK_WIDTH; // in

        RobotConstants.MAX_VEL = MAX_VEL;
        RobotConstants.MAX_ACCEL = MAX_ACCEL;
        RobotConstants.MAX_ANG_VEL = MAX_ANG_VEL;
        RobotConstants.MAX_ANG_ACCEL = MAX_ANG_ACCEL;

        RobotConstants.frontLeftFeedForward[0] = frontLeftKS;
        RobotConstants.frontLeftFeedForward[1] = frontLeftKV;
        RobotConstants.frontLeftFeedForward[2] = frontLeftKA;
        RobotConstants.frontRightFeedForward[0] = frontRightKS;
        RobotConstants.frontRightFeedForward[1] = frontRightKV;
        RobotConstants.frontRightFeedForward[2] = frontRightKA;
        RobotConstants.rearLeftFeedForward[0] = rearLeftKS;
        RobotConstants.rearLeftFeedForward[1] = rearLeftKV;
        RobotConstants.rearLeftFeedForward[2] = rearLeftKA;
        RobotConstants.rearRightFeedForward[0] = rearRightKS;
        RobotConstants.rearRightFeedForward[1] = rearRightKV;
        RobotConstants.rearRightFeedForward[2] = rearRightKA;

        RobotConstants.VELO_KP = VELO_KP;
        RobotConstants.VELO_KI = VELO_KI;
        RobotConstants.VELO_KD = VELO_KD;
        RobotConstants.minIntegralBound = minIntegralBound;
        RobotConstants.maxIntegralBound = maxIntegralBound;

        RobotConstants.TICKS_PER_REV = TICKS_PER_REV;
        RobotConstants.MAX_RPM = MAX_RPM;

        RobotConstants.DEFAULT_SPEED_PERC = DEFAULT_SPEED_PERC;
        RobotConstants.SLOW_SPEED_PERC = SLOW_SPEED_PERC;
        RobotConstants.FAST_SPEED_PERC = FAST_SPEED_PERC;


        robot = new CenterStageRobot(hardwareMap, RobotConstants, telemetry, driverOp, toolOp, RobotEx.OpModeType.TELEOP, true);
    }
}