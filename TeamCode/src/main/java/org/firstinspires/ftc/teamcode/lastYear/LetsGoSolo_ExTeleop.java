package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LetsGoSolo_ExTeleop")
@Disabled
public class LetsGoSolo_ExTeleop extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;
    private DcMotor back_left;
    private Servo armL;
    private DcMotor caroswl;
    private DcMotor cascade;
    private DcMotor intake;

@Override
    public void runOpMode() {
        float LStickY;
        double speedMultiplier;
        boolean PilotIs1;
        float LStickX;
        float RStickX;
        float Forward;
        float turn;
        float Strafe;

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        armL = hardwareMap.get(Servo.class, "armL");
        caroswl = hardwareMap.get(DcMotor.class, "caroswl");
        cascade = hardwareMap.get(DcMotor.class, "cascade");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Put initialization blocks here.
        armL.scaleRange(0, 1);
        waitForStart();
        speedMultiplier = 1;
        if (opModeIsActive()) {
            PilotIs1 = true;
            // fr gamepad 1 left y - gamepad left x 0 gamepad 1 right x fl
            // -gamepad1 left y - gamepad 1 left x - gamepad 1 right x br
            // = gaempad 1 left y + gamepad 1 left x - gamepad 1 right x bl
            // =- -gamepad 1 left y + gamepad 1 left x - gamepad 1 right x
            while (opModeIsActive()) {
                if (PilotIs1) {
                    if (gamepad2.right_stick_button) {
                        PilotIs1 = false;
                    }
                    front_left.setPower(0);
                    back_right.setPower(0);
                    front_right.setPower(0);
                    back_left.setPower(0);
                    LStickY = gamepad1.left_stick_y;
                    LStickX = gamepad1.left_stick_x;
                    RStickX = gamepad1.right_stick_x;
                    Forward = gamepad1.left_stick_y;
                    turn = gamepad1.right_stick_x;
                    Strafe = gamepad1.left_stick_x;
                    if (gamepad1.a == true) {
                        caroswl.setPower(0.25);
                        for (int count = 0; count < 150; count++) {
                            caroswl.setPower(caroswl.getPower() + 0.01);
                            sleep(10);
                        }
                        caroswl.setPower(1);
                        sleep(250);
                    } else {
                        caroswl.setPower(0);
                    }
                    if (gamepad1.b == true) {
                        caroswl.setPower(-0.9);
                        telemetry.update();
                        sleep(800);
                        caroswl.setPower(-1);
                        telemetry.update();
                        sleep(1200);
                        caroswl.setPower(0.1);
                        telemetry.update();
                        sleep(1);
                    } else {
                        caroswl.setPower(0);
                    }
                    if (gamepad1.right_trigger > 0.01) {
                        cascade.setPower(gamepad1.right_trigger);
                    } else if (gamepad1.left_trigger > 0.01) {
                        cascade.setPower(gamepad1.left_trigger / -3);
                    } else {
                        cascade.setPower(0);
                    }
                    if (gamepad1.left_bumper) {
                        intake.setPower(1);
                    } else if (gamepad1.right_bumper) {
                        intake.setPower(-1);
                    } else {
                        intake.setPower(0);
                    }
                    if (gamepad1.dpad_right == true) {
                        speedMultiplier = 0.1;
                    }
                    if (gamepad1.dpad_left == true) {
                        speedMultiplier = 0.75;
                    }
                    if (gamepad1.dpad_down == true) {
                        speedMultiplier = 0.3;
                    }
                    if (gamepad1.dpad_up == true) {
                        speedMultiplier = 1;
                    }
                    back_left.setPower(((1 * gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * speedMultiplier);
                    back_right.setPower(((-1 * gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * speedMultiplier);
                    front_left.setPower(((1 * gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * speedMultiplier);
                    front_right.setPower(((-1 * gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * speedMultiplier);
                    telemetry.update();
                } else {
                    if (gamepad1.right_stick_button) {
                        PilotIs1 = true;
                    }
                    front_left.setPower(0);
                    back_right.setPower(0);
                    front_right.setPower(0);
                    back_left.setPower(0);
                    LStickY = gamepad2.left_stick_y;
                    LStickX = gamepad2.left_stick_x;
                    RStickX = gamepad2.right_stick_x;
                    Forward = gamepad2.left_stick_y;
                    turn = gamepad2.right_stick_x;
                    Strafe = gamepad2.left_stick_x;
                    if (gamepad2.a == true) {
                        caroswl.setPower(0.25);
                        for (int count2 = 0; count2 < 150; count2++) {
                            caroswl.setPower(caroswl.getPower() + 0.01);
                            sleep(10);
                        }
                        caroswl.setPower(1);
                        sleep(250);
                    } else {
                        caroswl.setPower(0);
                    }
                    if (gamepad2.b == true) {
                        caroswl.setPower(-0.9);
                        telemetry.update();
                        sleep(800);
                        caroswl.setPower(-1);
                        telemetry.update();
                        sleep(1200);
                        caroswl.setPower(0.1);
                        telemetry.update();
                        sleep(1);
                    } else {
                        caroswl.setPower(0);
                    }
                    if (gamepad1.a == true) {
                        caroswl.setPower(0.25);
                        for (int count3 = 0; count3 < 150; count3++) {
                            caroswl.setPower(caroswl.getPower() + 0.01);
                            sleep(10);
                        }
                        caroswl.setPower(1);
                        sleep(250);
                    } else {
                        caroswl.setPower(0);
                    }
                    if (gamepad1.b == true) {
                        caroswl.setPower(-0.9);
                        telemetry.update();
                        sleep(800);
                        caroswl.setPower(-1);
                        telemetry.update();
                        sleep(1200);
                        caroswl.setPower(0.1);
                        telemetry.update();
                        sleep(1);
                    } else {
                        caroswl.setPower(0);
                    }
                    if (gamepad2.dpad_right == true) {
                        speedMultiplier = 0.1;
                    }
                    if (gamepad2.dpad_left == true) {
                        speedMultiplier = 0.75;
                    }
                    if (gamepad2.dpad_down == true) {
                        speedMultiplier = 0.3;
                    }
                    if (gamepad2.dpad_up == true) {
                        speedMultiplier = 1;
                    }
                    back_left.setPower(((1 * gamepad2.left_stick_y + gamepad2.left_stick_x) - gamepad2.right_stick_x) * speedMultiplier);
                    back_right.setPower(((-1 * gamepad2.left_stick_y + gamepad2.left_stick_x) - gamepad2.right_stick_x) * speedMultiplier);
                    front_left.setPower(((1 * gamepad2.left_stick_y - gamepad2.left_stick_x) - gamepad2.right_stick_x) * speedMultiplier);
                    front_right.setPower(((-1 * gamepad2.left_stick_y - gamepad2.left_stick_x) - gamepad2.right_stick_x) * speedMultiplier);
                    telemetry.update();
                }
            }
        }
    }
}