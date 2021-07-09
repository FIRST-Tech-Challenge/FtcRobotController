package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testProgram8")
public class testProgram8 extends LinearOpMode {

    DcMotor FR = null;
    DcMotor FL = null;
    DcMotor BR = null;
    DcMotor BL = null;
    DcMotor shooter = null;
    DcMotor wobbleArm = null;
    DcMotor collectionMechanism = null;
    DcMotor neat = null;
    Servo ringPusher = null;
    Servo wobbleGoalGrabber = null;

    public void runOpMode() {
        telemetry.addData("status", "Initialization Is good");
        telemetry.update();

        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        collectionMechanism = hardwareMap.get(DcMotor.class, "collectionMechanism");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        neat = hardwareMap.get(DcMotor.class, "neat");

        wobbleGoalGrabber = hardwareMap.get(Servo.class, "wobbleGoalGrabber");
        ringPusher = hardwareMap.get(Servo.class, "ringPusher");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        boolean intaketoggle = false;
        while (opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_x, -1 * gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-1 * gamepad1.left_stick_y, gamepad1.left_stick_x) + -20;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            final double wobblepower = -1 * gamepad2.left_stick_y;

            //neutral position is 0.7, ring push position is 0.1
            if (gamepad2.b) {
                shooter.setPower(-0.5);
            } else {
                shooter.setPower(0);
            }

            if (gamepad2.a) {
                ringPusher.setPosition(0.1);
            } else {
                ringPusher.setPosition(0.7);
            }

            if (gamepad2.y) {
                wobbleGoalGrabber.setPosition(0.1);
            } else {
                wobbleGoalGrabber.setPosition(0.7);
            }

            if (gamepad2.x) {
               intaketoggle = !intaketoggle;}

            if (intaketoggle = true) {
            collectionMechanism.setPower(1);
                neat.setPower(1); }
            if (intaketoggle = false){
                collectionMechanism.setPower(0);
                neat.setPower(0);
            }
            // if left trigger not pressed, run eveything full power
            if (gamepad1.left_trigger < 0.1){
                FL.setPower(v1);
                FR.setPower(v2);
                BL.setPower(v3);
                BR.setPower(v4);
                wobbleArm.setPower(wobblepower);
                // if left trigger pressed, run eveything at lower power
            } else if (gamepad1.left_trigger > 0.1) {
                FL.setPower(v1 / 8);
                FR.setPower(v2 / 8);
                BL.setPower(v3 / 8);
                BR.setPower(v4 / 8);
                wobbleArm.setPower(wobblepower/2);
            }
            telemetry.addData("status", "motor FL (v1): " + v1);
            telemetry.addData("status", "motor FR (v2): " + v2);
            telemetry.addData("status", "motor BL (v3): " + v3);
            telemetry.addData("status", "motor FL (v4): " + v4);
            telemetry.addData("status", "r" + r);
            telemetry.addData("status", "robot angle: " + robotAngle);
            telemetry.update();
        }
    }
}