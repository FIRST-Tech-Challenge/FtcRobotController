package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FredTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            double powerForward = -gamepad1.left_stick_y / 2;
            double powerTurn = gamepad1.right_stick_x / 2;
            double mecanumMovement = gamepad1.left_stick_x / 2;
            //double armMovement = gamepad1.right_stick_y;


            double fLeftPower = (powerForward + powerTurn + mecanumMovement);
            double fRightPower = (powerForward - powerTurn - mecanumMovement);
            double bLeftPower = (powerForward + powerTurn - mecanumMovement);
            double bRightPower = (powerForward - powerTurn + mecanumMovement);

            double maxPower = maxAbsPower(fLeftPower,fRightPower,bLeftPower,bRightPower);

            if (Math.abs(maxPower) > 1) {
                double scale = Math.abs(maxPower);

                fLeftPower /= scale;
                fRightPower /= scale;
                bLeftPower /= scale;
                bRightPower /= scale;
            }

            //if (gamepad1.left_bumper == true) {
            //servoOnePos += 0.025;
            //servoTwoPos -= 0.025;
            //}

            //if (gamepad1.right_bumper == true) {
            //servoOnePos -= 0.025;
            //servoTwoPos += 0.025;
            //}


            fLeft.setPower(fLeftPower);
            fRight.setPower(fRightPower);
            bLeft.setPower(bLeftPower);
            bRight.setPower(bRightPower);
            //arm.setPower(armMovement);
            //servo1.setPosition(servoOnePos);
            //servo2.setPosition(servoTwoPos);

        }
    }

    private double maxAbsPower(double a, double... others) {

        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return max;

    }
}

