package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="ColorSensorTeleop")
public class ColorSensorTeleop extends OpMode {


    public DcMotor backleft;
    public DcMotor frontleft;
    public DcMotor backright;
    public DcMotor frontright;
    public DcMotor lift;
    public DcMotor leftgrabber;
    public DcMotor rightgrabber;

    public ColorSensor color = null;



    public double speedMode = 1;
    public boolean xIsHeld = false;
    public boolean bIsHeld = false;
    public boolean dpadLeftIsHeld = false;
    public boolean dpadRightIsHeld = false;

    public double max_lift = 95;
    public double min_lift = -1;




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
        leftgrabber = hardwareMap.get(DcMotor.class, "leftgrabber");
        rightgrabber = hardwareMap.get(DcMotor.class, "rightgrabber");
        color = hardwareMap.get(ColorSensor.class, "sensor_color");



        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        leftgrabber.setDirection(DcMotor.Direction.FORWARD);
        rightgrabber.setDirection(DcMotor.Direction.FORWARD);


        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftgrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightgrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftgrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightgrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftgrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightgrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        backleft.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        frontright.setPower(0);
        lift.setPower(0);
        leftgrabber.setPower(0);
        rightgrabber.setPower(0);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }


    @Override
    public void loop() {

        telemetry.addData("Red", color.red());
        telemetry.update();



    }
}
