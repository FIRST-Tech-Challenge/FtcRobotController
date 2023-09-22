package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class XDrive extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor northWheel;
    private DcMotor southWheel;
    private DcMotor westWheel;
    private DcMotor eastWheel;
    private DcMotor neck;
    private DcMotor shoulder;
    private Servo wrist;
    private Servo leftGrip;
    private Servo rightGrip;

    @Override
    public void RunOpMode(){

        imu = hardwareMap.get(Gyroscope.class, "imu");
        northWheel = hardwareMap.get(DcMotor.class, "northWheel");
        southWheel = hardwareMap.get(DcMotor.class, "southWheel");
        westWheel = hardwareMap.get(DcMotor.class, "westWheel");
        eastWheel = hardwareMap.get(DcMotor.class, "eastWheel");
        neck = hardwareMap.get(DcMotor.class, "neck");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("status", "running");
            telemetry.update();
        }
    }

}
