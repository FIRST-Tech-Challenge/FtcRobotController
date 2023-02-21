package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TeleOP")
public class TeleOP extends OpMode {


    public DcMotor backleft;
    public DcMotor frontleft;
    public DcMotor backright;
    public DcMotor frontright;
    public DcMotor lift;
    public DcMotor leftgrabber;
    public DcMotor rightgrabber;


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



//        lift.setTargetPosition(95);


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

        telemetry.addData("Lift", lift.getCurrentPosition());
        telemetry.update();

        //Slow Mode Code for a and b keys
        if (gamepad1.a) {
            speedMode = .4;
        }

            //Slow Mode Code for a and b keys


            //Turtle Mode code for bumpers
            if (gamepad1.right_bumper == true) {
                speedMode = .20;
            }else if(gamepad1.right_bumper == false) {
                speedMode = .5;
            }

            if (gamepad1.right_trigger >= .5) {
                speedMode = 1;
            }
            //Slow Mode Code for bumpers


            double stopBuffer = 0; //Not currently Implemented


            //Drive Train Code
            double forward = speedMode * Math.pow(gamepad1.left_stick_y, 3);
            double right = -speedMode * Math.pow(gamepad1.left_stick_x, 3);
            double turn = -speedMode * Math.pow(gamepad1.right_stick_x, 3);

            double leftFrontPower = forward + right + turn;
            double leftBackPower = forward - right + turn;
            double rightFrontPower = forward - right - turn;
            double rightBackPower = forward + right - turn;
            double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

            boolean needToScale = false;
            for (double power : powers) {
                if (Math.abs(power) > 1) {
                    needToScale = true;
                    break;
                }
            }
            if (needToScale) {
                double greatest = 0;
                for (double power : powers) {
                    if (Math.abs(power) > greatest) {
                        greatest = Math.abs(power);
                    }
                }
                leftFrontPower /= greatest;
                leftBackPower /= greatest;
                rightFrontPower /= greatest;
                rightBackPower /= greatest;
            }

            boolean stop = true;
            for (double power : powers) {
                if (Math.abs(power) > stopBuffer) {
                    stop = false;
                    break;
                }
            }
            if (stop) {
                leftFrontPower = 0;
                leftBackPower = 0;
                rightFrontPower = 0;
                rightBackPower = 0;
            }

            frontleft.setPower(leftFrontPower);
            backleft.setPower(leftBackPower);
            frontright.setPower(rightFrontPower);
            backright.setPower(rightBackPower);
            //Drive Train Code


            //lift code
            if (gamepad2.left_stick_y >= 0.3) {
                lift.setPower(1);
            } else if (gamepad2.left_stick_y <= -0.3) {
                lift.setPower(-1);
            } else {
                lift.setPower(0);
            }


            //lift limit
            if(lift.getCurrentPosition() <= -2800){
                lift.setPower(0);
            }

            //lift quick
//            if(gamepad2.dpad_up == true){
//                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift.setTargetPosition(33);
//            }
//            if(gamepad2.dpad_left == true){
//            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lift.setTargetPosition(23);
//            }
//            if(gamepad2.dpad_down == true){
//            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lift.setTargetPosition(15);
//            }




            //wormhole in
            if (gamepad2.b) {
                leftgrabber.setPower(1);
                rightgrabber.setPower(-1);

                //wormhole out
            } else if (gamepad2.x) {
                leftgrabber.setPower(-1);
                rightgrabber.setPower(1);

            } else {
                leftgrabber.setPower(0);
                rightgrabber.setPower(0);

            }



        }
    }
