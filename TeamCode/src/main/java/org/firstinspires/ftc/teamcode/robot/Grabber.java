package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public class Grabber {
    private Telemetry telemetry;
    private Servo grabber;
    public final String SYSTEM_NAME = "GRABBER";
    public final String OPEN_STATE = "OPEN";
    public final String CLOSED_STATE = "CLOSED";
    public final double OPEN_VALUE = 0.4;
    public final double CLOSED_VALUE = 0.685;

    public Grabber(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        grabber = hwMap.servo.get("Grabber");


    }


    public void setState(String position){

        telemetry.addData("GrabberState", position);
        switch (position) {
            case OPEN_STATE: {
                grabber.setPosition(OPEN_VALUE);
                break;
            }
            case CLOSED_STATE: {
                grabber.setPosition(CLOSED_VALUE);
                break;
            }
        }
    }
}