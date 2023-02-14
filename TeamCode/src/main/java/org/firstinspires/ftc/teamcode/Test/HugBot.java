package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="HugBot")
public class HugBot extends OpMode {


    public DcMotor backleft;
    public DcMotor frontleft;
    public DcMotor backright;
    public DcMotor frontright;
    public DcMotor lift;

    public Servo leftgrabber;
    public Servo rightgrabber;


    public double speedMode = 1;
    public boolean xIsHeld = false;
    public boolean bIsHeld = false;
    public boolean dpadLeftIsHeld = false;
    public boolean dpadRightIsHeld = false;


    @Override
    public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        //Hardware map
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        lift = hardwareMap.get(DcMotor.class, "lift");
        leftgrabber = hardwareMap.get(Servo.class, "leftgrabber");
        rightgrabber = hardwareMap.get(Servo.class, "rightgrabber");


        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        leftgrabber.setDirection(Servo.Direction.FORWARD);
        rightgrabber.setDirection(Servo.Direction.FORWARD);


        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        backleft.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        frontright.setPower(0);
        lift.setPower(0);
        leftgrabber.setPosition(0);
        rightgrabber.setPosition(0);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }


    @Override
    public void loop() {

        if(gamepad1.left_stick_y >= .3){
            backleft.setPower(.5);
            frontleft.setPower(.5);
            backright.setPower(.5);
            frontright.setPower(.5);
        }else if(gamepad1.left_stick_y <= -.3){
            backleft.setPower(-.5);
            frontleft.setPower(-.5);
            backright.setPower(-.5);
            frontright.setPower(-.5);
        }else{
            backleft.setPower(0);
            frontleft.setPower(0);
            backright.setPower(0);
            frontright.setPower(0);
        }

        //lift code
        if (gamepad1.dpad_up =true) {
            lift.setPower(.8);
        } else if (gamepad1.dpad_down = true) {
            lift.setPower(-.8);
        } else {
            lift.setPower(0);
        }

        //hug code
        if (gamepad1.a =true) {
            leftgrabber.setPosition(180);
            leftgrabber.setPosition(180);
        }

        if (gamepad1.b =true) {
            leftgrabber.setPosition(0);
            leftgrabber.setPosition(0);
        }


    }
}
