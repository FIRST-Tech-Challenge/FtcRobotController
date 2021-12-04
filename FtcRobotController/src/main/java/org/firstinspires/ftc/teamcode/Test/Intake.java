package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class Intake extends LinearOpMode {
    DcMotor motor2;
    boolean previousInput = false;
    boolean intakeToggle = false;

    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {

            motor2 = hardwareMap.get(DcMotor.class, "motorIntake");

            if (gamepad1.right_bumper == true) {

                motor2.setPower(0.3);

            } else {
                motor2.setPower(0);
            }


            if (gamepad1.right_bumper == true && previousInput == false) {
                intakeToggle = !intakeToggle;
                previousInput = true;
            }

            previousInput = gamepad1.right_bumper;

            if (intakeToggle == true) {
                motor2.setPower(0.3);
            }
            else {
                motor2.setPower(0);
            }


        }
    }
}
