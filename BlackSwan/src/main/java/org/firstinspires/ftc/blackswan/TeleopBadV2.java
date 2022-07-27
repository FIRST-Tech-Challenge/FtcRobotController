package org.firstinspires.ftc.blackswan;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleopBadV2")

public class TeleopBadV2 extends LinearOpMode {

    double MAX_SPEED = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft, backLeft, frontRight, backRight, slide, carousel, intake;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        carousel = hardwareMap.get(DcMotor.class, "carousel");

        slide = hardwareMap.get(DcMotor.class, "slide");

        intake = hardwareMap.get(DcMotor.class, "intake");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        //telemetry testing delete later!!!
        String detection = "none";
        while (opModeIsActive()) {
            //turn with right stick
//            telemetry.addData("left stick value x", gamepad1.left_stick_x);
//            telemetry.addData("left stick value y", gamepad1.left_stick_y);
            telemetry.addData("left stick value x", gamepad2.left_stick_x);
            telemetry.addData("left stick value y", gamepad2.left_stick_y);
            telemetry.addData("detection", detection);
            telemetry.update();
            if (gamepad1.right_stick_x > 0.1) {
                telemetry.addData("positive", gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
            } else if (gamepad1.right_stick_x < -0.1) {
                telemetry.addData("negative", gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
            } else if (gamepad1.left_stick_x < -0.25 && gamepad1.left_stick_y < -0.25) {
                //move UpLeft
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(0);
                frontLeft.setPower(0);
                detection = "UpLeft";
            } else if (gamepad1.left_stick_x > 0.25 && gamepad1.left_stick_y < -0.25) {
                //move UpRight
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(0);
                frontRight.setPower(0);
                detection = "UpRight";
            } else if (gamepad1.left_stick_x > 0.25 && gamepad1.left_stick_y > 0.25) {
                //move DownRight
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(0);
                frontLeft.setPower(0);
                detection = "DownRight";
            } else if (gamepad1.left_stick_x < -0.25 && gamepad1.left_stick_y > 0.25) {
                //move DownLeft
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(0);
                frontRight.setPower(0);
                detection = "DownLeft";
            } else if (gamepad1.left_stick_y > 0.1) {
                //move Down
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                detection = "Down";
            } else if (gamepad1.left_stick_y < -0.1) {
                //move Up
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                detection = "Up";
            } else if (gamepad1.left_stick_x < -0.1) {
                //move Right
                frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
                detection = "Right";
            } else if (gamepad1.left_stick_x > 0.1) {
                //move Left
                frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
                detection = "Left";
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                detection = "None";
            }
            if (gamepad2.dpad_up) { //up
                slide.setTargetPosition(1250);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(.5);

            }
            if (gamepad2.dpad_left) { //middle
                slide.setTargetPosition(825);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(.5);

            }
            if (gamepad2.dpad_right) { //low
                slide.setTargetPosition(500);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(.5);

            }

            if (gamepad2.dpad_down) {
                slide.setTargetPosition(75);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(.3);
//                while (arm.isBusy() && opModeIsActive()) {
//
//                }
//                arm.setPower(0);
//                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            if (gamepad2.right_trigger > 0.1) {
                intake.setPower(-1);
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }


//this is to set the intake to cover the element inside
//            if (gamepad2.a) {
//                intake.setTargetPosition(-356);
//                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                intake.setPower(0.8);
//                while (intake.isBusy() && opModeIsActive()) {
//                }
//
//                telemetry.update();
//            }

            turnDuck(carousel);


        }
    }

    protected void turnDuck(DcMotor carousel){
        if(gamepad2.right_bumper){
            carousel.setPower(-0.5 );
        } else  if (gamepad2.left_bumper){
            carousel.setPower(0.5);
        }else {
            carousel.setPower(0);
        }
    }

}

