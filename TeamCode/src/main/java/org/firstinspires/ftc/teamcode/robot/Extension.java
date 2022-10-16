package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Extension {
    private Telemetry telemetry;
    public DcMotorEx extensionMotor;

    static final double MM_TO_INCHES = 0.0393700787;

    static final double MOVE_OUT_SPEED = 0.5;
    static final double MOVE_BACK_SPEED = 0.5;

    static final double MINIMUM_CLEARANCE_DISTANCE = 95.875 * MM_TO_INCHES;

    public final double EXTENSION_POSITION_RESET = 0;
    //public final double EXTENSION_POSITION_MAXIMUM = inches of maximum extend
}
