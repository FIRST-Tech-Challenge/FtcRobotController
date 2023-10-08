package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//add any test stuff you need to do here
@TeleOp
public class testTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor flippyThingy = hardwareMap.dcMotor.get("flipper");
        flippyThingy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flippyThingy.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a == true) {
                intake.setPower(0.6);

            } else if (gamepad1.b == true) {
                intake.setPower(-0.6  );
            } else {
                intake.setPower(0);
            }

            if (gamepad1.x == true) {
                flippyThingy.setPower(0.6);

            } else if (gamepad1.y == true) {
                flippyThingy.setPower(-0.6);
            } else {
                flippyThingy.setPower(0);
            }

            double forwardMovement = -gamepad1.left_stick_y;
            double turnMovement = gamepad1.right_stick_x;
            double mecanumMovement = -gamepad1.left_stick_x;

            double fLeftPower = forwardMovement + turnMovement + mecanumMovement;
            double bLeftPower = forwardMovement + turnMovement - mecanumMovement;
            double fRightPower = forwardMovement - turnMovement - mecanumMovement;
            double bRightPower = forwardMovement - turnMovement + mecanumMovement;

            double maxPower = maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

            if (Math.abs(maxPower) > 1) {
                double scale = Math.abs(maxPower);

                fLeftPower /= scale;
                bLeftPower /= scale;
                fRightPower /= scale;
                bRightPower /= scale;
            }

            fLeft.setPower(fLeftPower);
            bLeft.setPower(bLeftPower);
            fRight.setPower(fRightPower);
            bLeft.setPower(bRightPower);
        }
    }

    private double maxAbsValueDouble(double a, double... others) {
        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return max;
    }
}
