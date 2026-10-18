package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TestMotors extends LinearOpMode {

    DcMotor outtake;
    DcMotor intake;
    CRServo left;
    CRServo right;
    DcMotor transfer;
    @Override
    public void runOpMode() throws InterruptedException {
        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        outtake.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        if(isStopRequested()) return;

        double power;

        power = 0;

        boolean leftBumper = false;
        boolean rightBumper = false;

        while(opModeIsActive()){


            if(gamepad1.left_bumper && !leftBumper) {
                power += 0.05;

            } else if(gamepad1.right_bumper && !rightBumper) {
                power -= 0.05;
            }

            leftBumper = gamepad1.left_bumper;
            rightBumper = gamepad1.right_bumper;
            if(gamepad1.a) {
                outtake.setPower(power);
                transfer.setPower(1);
            } else if(gamepad1.b) {
                outtake.setPower(-power);
                transfer.setPower(-1);
            } else {
                outtake.setPower(0);
                transfer.setPower(0);
            }
            if(gamepad1.x) {
                intake.setPower(1);
                left.setPower(1);
                right.setPower(-1);
            } else if(gamepad1.y) {
                intake.setPower(-1);
                left.setPower(-1);
                right.setPower(1);
            } else {
                intake.setPower(0);
                left.setPower(0);
                right.setPower(0);
            }

            telemetry.addData("Power", power);
            telemetry.update();
        }

    }
}
