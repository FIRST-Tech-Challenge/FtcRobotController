package org.firstinspires.ftc.teamcode.Richard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RichardDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
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

            fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive()) {
                telemetry.addLine(String.valueOf(fLeft.getCurrentPosition()));
                telemetry.update();
            }



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