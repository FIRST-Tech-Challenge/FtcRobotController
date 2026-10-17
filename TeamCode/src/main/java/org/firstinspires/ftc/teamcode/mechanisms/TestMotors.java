package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestMotors extends LinearOpMode {

    DcMotor outtake;
    DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        outtake.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
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
            } else if(gamepad1.b) {
                outtake.setPower(-power);
            } else {
                outtake.setPower(0);
            }
            if(gamepad1.x) {
                intake.setPower(power);
            } else if(gamepad1.y) {
                intake.setPower(-power);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("Power", power);
            telemetry.update();
        }

    }
}
