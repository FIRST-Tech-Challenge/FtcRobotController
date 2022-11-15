package org.firstinspires.ftc.teamcode.lastYear;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="José", group="Linear Opmode")
public class José extends LinearOpMode {
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor backLeft;
        private DcMotor backRight;
//        private DcMotor cascade;
//        private DcMotor intake;

        @Override
        public void runOpMode() {
                frontLeft = hardwareMap.get(DcMotor.class, "front_left");
                frontRight = hardwareMap.get(DcMotor.class, "front_right");
                backLeft = hardwareMap.get(DcMotor.class, "back_left");
                backRight = hardwareMap.get(DcMotor.class, "back_right");
//                cascade = hardwareMap.get(DcMotor.class, "cascade");
//                intake = hardwareMap.get(DcMotor.class, "intake");

                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setDirection(DcMotor.Direction.FORWARD);

//                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                telemetry.addData("Initialized", "Ready to start");
                waitForStart();

                while (opModeIsActive()) {
/*
  Drivetrain Options
 */
//                        double speed;
//                        speed = 1;
//                        backLeft.setPower(((1 * gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * speed);
//                        backRight.setPower(((-1 * gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * speed);
//                        frontLeft.setPower(((1 * gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * speed);
//                        frontRight.setPower(((-1 * gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * speed);
//
//                        double speed = -gamepad1.left_stick_y;
//                        double turn = gamepad1.right_stick_x;
//                        double strafe = gamepad1.left_stick_x;
//                        frontLeft.setPower(speed + turn + strafe);
//                        frontRight.setPower(speed - turn - strafe);
//                        backLeft.setPower(speed + turn - strafe);
//                        backRight.setPower(speed - turn + strafe);
//
                        double x = gamepad1.left_stick_x;
                        double y = gamepad1.left_stick_y*-1;
                        double turn = gamepad1.right_stick_x;
                        double theta = Math.atan2(y, x);
                        double power = Math.hypot(x, y);
                        double sin = Math.sin(theta - Math.PI/4);
                        double cos = Math.cos(theta - Math.PI/4);
                        double max = Math.max(Math.abs(sin), Math.abs(cos));
                        frontLeft.setPower(power * cos/max + turn);
                        frontRight.setPower(power * sin/max - turn);
                        backLeft.setPower(power * sin/max + turn);
                        backRight.setPower(power * cos/max - turn);
                        if ((power + Math.abs(turn)) > 1) {
                                frontLeft.setPower((frontLeft.getPower()) / power + turn);
                                frontRight.setPower((frontRight.getPower()) / power + turn);
                                backLeft.setPower((backLeft.getPower()) / power + turn);
                                backRight.setPower((backRight.getPower()) / power + turn);
                        }
/*
  Arm Cascade Options
  */
//                        if (gamepad1.right_trigger > 0.01) {
//                                cascade.setPower(gamepad1.right_trigger);
//                        } else if (gamepad1.left_trigger > 0.01) {
//                                cascade.setPower(gamepad1.left_trigger / -3);
//                        } else {
//                                cascade.setPower(0);
//                        }
//
//                        if (gamepad1.right_trigger > 0.01) {
//                                cascade.setPower(gamepad1.right_trigger);
//                        } else if (gamepad1.left_trigger > 0.01) {
//                                cascade.setPower(gamepad1.left_trigger / -3);
//                        } else {
//                                cascade.setPower(0.1);
//                        }
//
//                        Something having to do with encoders...
/*
  Intake
  */
//                        if (gamepad1.left_bumper) {
//                                intake.setPower(1);
//                        } else if (gamepad1.right_bumper) {
//                                intake.setPower(-1);
//                        } else {
//                                intake.setPower(0);
//                        }


                        telemetry.update();
                }
        }

        public void strafeLeft(double power) {
                frontRight.setPower(power);
                backRight.setPower(power*-1);
                frontLeft.setPower(power*-1);
                backLeft.setPower(power);
        }

        public void strafeRight(double power) {
                frontRight.setPower(power*-1);
                backRight.setPower(power);
                frontLeft.setPower(power);
                backLeft.setPower(power*-1);
        }

        public void backward(double power) {
                frontRight.setPower(power*-1);
                backRight.setPower(power*-1);
                frontLeft.setPower(power*-1);
                backLeft.setPower(power*-1);
        }

        public void forward(double power) {
                frontRight.setPower(power);
                backRight.setPower(power);
                frontLeft.setPower(power);
                backLeft.setPower(power);
        }

        public void spinLeft(double power) {
                frontRight.setPower(power*-1);
                backRight.setPower(power*-1);
                frontLeft.setPower(power);
                backLeft.setPower(power);
        }

        public void spinRight(double power) {
                frontRight.setPower(power);
                backRight.setPower(power);
                frontLeft.setPower(power*-1);
                backLeft.setPower(power*-1);
        }

        public void forward() {
                forward(1);
        }

        public void strafeLeft() {
                strafeLeft(1);
        }

        public void strafeRight() {
                strafeRight(1);
        }

        public void backward() {
                backward(1);
        }

        public void spinLeft() {
                spinLeft(1);
        }

        public void spinRight() {
                spinRight(1);
        }
        //Space Pianist was here
}
