package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//how to import these packages?

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.MotionProfiler;

public class Slides {
    public final DcMotorEx motor_1;
    private static double p = 0.015, i = 0 , d = 0, f = 0;
    //4410 code has staticF, not sure what that is. Look into it later

    //change values in slidesPosition based on the number of stages in the slides
    public enum slidesPosition{
        GROUND,
        LOW,
        MID,
        HIGH
    }

    private slidesPosition position = slidesPosition.GROUND;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1, powerMin = 0.1;
    //change these values based on what they actually are
    private double manualPower = 0;

    public static int MAXHEIGHT = -2000, top = -1700, topTeleOp = -1750, mid = -980, low = -300, ground = 0, move_up_inc = 100, move_down_dec = 300;
    //change values based on what they actually are

    private final OpMode opMode;
    private double target = 0;
    private boolean goingDown = false;
    private double profile_init_time = 0;
    private MotionProfiler profiler = new MotionProfiler(30000, 20000);

}
