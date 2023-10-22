package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, this, telemetry);

        robot.setUpDrivetrainMotors();
        double forwardBackward = gamepad1.left_stick_y * -0.5;
        double turning = gamepad1.right_stick_x * 0.5;
        double mecanuming = gamepad1.left_stick_x * 0.5;

        double fLeftPower = forwardBackward + turning + mecanuming;
        double fRightPower = forwardBackward - turning - mecanuming;
        double bLeftPower = forwardBackward + turning - mecanuming;
        double bRightPower = forwardBackward - turning + mecanuming;



        robot.setUpIntakeOuttake();



        double flipperPos = 0.3;
        double armPos = 0.5;

        double remainingDistanceHigh = 300 - robot.lsFront.getCurrentPosition();
        double remainingDistanceMid = 200 - robot.lsFront.getCurrentPosition();
        double remainingDistanceLow = 100 - robot.lsFront.getCurrentPosition();
        double remainingDistanceZero = - robot.lsFront.getCurrentPosition();



        waitForStart();
        //gamepad 1
        while (opModeIsActive()) {

            forwardBackward = gamepad1.left_stick_y * -0.5;
            turning = gamepad1.right_stick_x * 0.5;
            mecanuming = gamepad1.left_stick_x * 0.5;

            fLeftPower = forwardBackward + turning + mecanuming;
            fRightPower = forwardBackward - turning - mecanuming;
            bLeftPower = forwardBackward + turning - mecanuming;
            bRightPower = forwardBackward - turning + mecanuming;

            double[] power = {
                    fLeftPower,
                    fRightPower,
                    bLeftPower,
                    bRightPower
            };

            telemetry.addData("fleft", power[0]);
            telemetry.addData("fright", power[1]);
            telemetry.addData("bleft", power[2]);
            telemetry.addData("bright", power[3]);

            power = robot.scalePowers(power);

            robot.setMotorPower(power);

            remainingDistanceHigh = 300 - robot.lsFront.getCurrentPosition();
            remainingDistanceMid = 200 - robot.lsFront.getCurrentPosition();
            remainingDistanceLow = 100 - robot.lsFront.getCurrentPosition();
            remainingDistanceZero = -robot.lsFront.getCurrentPosition();


            if (gamepad1.dpad_up && remainingDistanceHigh > 10) {

                robot.lsFront.setPower(remainingDistanceHigh*0.002);
                robot.lsBack.setPower(remainingDistanceHigh*0.002);

            } else if (gamepad1.dpad_right && remainingDistanceMid > 10) {

                robot.lsFront.setPower(remainingDistanceMid*0.002);
                robot.lsBack.setPower(remainingDistanceMid*0.002);

            } else if (gamepad1.dpad_left && remainingDistanceLow > 10) {

                robot.lsFront.setPower(remainingDistanceLow*0.002);
                robot.lsBack.setPower(remainingDistanceLow*0.002);

            } else if (gamepad1.dpad_down && remainingDistanceZero > 10) {

                robot.lsFront.setPower(remainingDistanceZero*0.002);
                robot.lsBack.setPower(remainingDistanceZero*0.002);

            }

            //TODO AANYA LAUNCHER GOES HERE!!!!!!

            //gamepad 2
            if (-gamepad2.left_stick_y > 0) {
                robot.lsBack.setPower(0.5);
                robot.lsFront.setPower(0.5);
            } else if (-gamepad2.left_stick_y < 0) {
                robot.lsBack.setPower(-0.5);
                robot.lsFront.setPower(-0.5);
            } else {
                robot.lsBack.setPower(0);
                robot.lsFront.setPower(0);
            }

            if (gamepad2.left_trigger >= 0.5) {
                robot.intake.setPower(1);
            } else if (gamepad2.left_bumper) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0);
            }

            if (gamepad2.right_trigger >= 0.5) {
                flipperPos += 0.0025;
            } else if (gamepad2.right_bumper) {
                flipperPos -= 0.0025;
            }

            if (-gamepad2.right_stick_y > 0) {
                armPos += 0.015;
            } else if (-gamepad2.right_stick_y < 0) {
                armPos -= 0.015;
            }

            robot.flipper.setPosition(flipperPos);
            robot.arm.setPosition(-armPos);




        }
    }
}
