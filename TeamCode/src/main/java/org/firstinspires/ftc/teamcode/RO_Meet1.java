package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(group = "FINALCODE")
public class RO_Meet1 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        // Declare our motors
        // Make sure your ID's match your configuration


        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor4");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor1");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor2");

//        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpRight");

//        DcMotorEx LeftViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//
//        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        RightViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /*
         Reverse the right side motors
         Reverse left motors if you are using NeveRests
         motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
         motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        */

        waitForStart();

        if (isStopRequested()) return;
//        long starttime = System.currentTimeMillis();


//        boolean useIncrements = false;
//        boolean useButtons = true;
//        int savedPosition = 0;
//        boolean goFast = false;
//        boolean goSlow = false;
//        boolean clawOpen = true;
//        long prevInterval = starttime;
//        int position = 0;
//        int prevposition = 0;
//        boolean clawfront = true;
        double y;
        double x;
        double rx;

        while (opModeIsActive()) {


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // servo2.setDirection(Servo.Direction.REVERSE);
            if (gamepad1.right_trigger > 0) {
                y = -gamepad1.left_stick_y; // Remember, this is reversed!
                x = gamepad1.left_stick_x; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
            } else if (gamepad1.left_trigger > 0) {
                y = 0.25 * -gamepad1.left_stick_y; // Remember, this is reversed!
                x = 0.25 * gamepad1.left_stick_x; // Counteract imperfect strafing
                rx = 0.35 * gamepad1.right_stick_x;
            } else {
                y = -0.5 * gamepad1.left_stick_y; // Remember, this is reversed!
                x = 0.5 * gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = 0.65 * gamepad1.right_stick_x;
            }

            /*
             Denominator is the largest motor power (absolute value) or 1
             This ensures all the powers maintain the same ratio, but only when
             at least one is out of the range [-1, 1]
            */
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            motorFrontLeft.setPower(-frontLeftPower);
            motorBackLeft.setPower(-backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("odometer middle pos", motorBackRight.getCurrentPosition());
            telemetry.addData("odometer right pos", motorFrontRight.getCurrentPosition());
            telemetry.addData("odometer left pos", motorBackLeft.getCurrentPosition());
            telemetry.update();


//// Viper Slide Code:
//
//
//            if(gamepad1.x && useButtons){
//                position = 1200;
//                goSlow = false;
//
//            }
//            if(gamepad1.y && useButtons) {
//                position =  2023;
//                goSlow = false;
//
//            }
//            if(gamepad1.b && useButtons) {
//                position = 2850;
//                goSlow = false;
//
//            }
//            if(gamepad1.a && useButtons) {
//                goSlow = false;
//                position = 0;
//            }
//
//            if (prevposition != position) {
//                RightViperSlide.setTargetPosition(position);
//                LeftViperSlide.setTargetPosition(-position);
//                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                prevposition = position;
//                RightViperSlide.setVelocity(4000);
//                LeftViperSlide.setVelocity(4000);
//            }
//
//
//            if (gamepad1.dpad_down) {
//                position = 100;
//                goSlow = true;
//            }
//            if (gamepad1.dpad_left) {
//                position = 200;
//                goSlow = true;
//            }
//            if (gamepad1.dpad_up) {
//                position = 300;
//                goSlow = true;
//            }
//            if (gamepad1.dpad_right) {
//                position = 420;
//                goSlow = true;
//            }
//
//            //Servo Code
//            if (gamepad1.right_bumper) {
//                if (clawOpen) {
//                    clawLeft.setPosition(0.0);
//                    clawRight.setPosition(0.7);
//                    if (position < 75) {
//                        position = 75;
//                    }
//                    TimeUnit.MILLISECONDS.sleep(500);
//                    clawOpen = false;
//                } else {
//                    clawLeft.setPosition(0.5);
//                    clawRight.setPosition(0.2);
//                    if (position <= 75) {
//                        position = 0;
//                    }
//                    TimeUnit.MILLISECONDS.sleep(500);
//                    clawOpen = true;
//                }
//            }
//        }
//        if (gamepad1.left_bumper) {
//            if (clawOpen) {
//                clawLeft.setPosition(1.1);
//                clawRight.setPosition(0.8);
//                if (position < 75) {
//                    position = 75;
//                }
//                TimeUnit.MILLISECONDS.sleep(500);
//                clawOpen = false;
//            } else {
//                clawLeft.setPosition(0.5);
//                clawRight.setPosition(0.2);
//                if (position <= 75) {
//                    position = 0;
//                }
//                TimeUnit.MILLISECONDS.sleep(500);
//                clawOpen = true;
//            }
//        }


        }
    }
}