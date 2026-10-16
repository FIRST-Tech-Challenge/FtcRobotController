package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestIntake extends LinearOpMode {

    DcMotor intake;
    CRServo intakeServo1;
    CRServo intakeServo2;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intakeServo1 = hardwareMap.get(CRServo.class, "servo1");
        intakeServo2 = hardwareMap.get(CRServo.class, "servo2");
        waitForStart();
        if(isStopRequested()) return;

        double power;

        power = 0;

        while(opModeIsActive()){
            if(gamepad1.left_bumper) {
                power += 0.1;
            } else if(gamepad1.right_bumper) {
                power -= 0.1;
            }
            if(gamepad1.a) {
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
            } else if(gamepad1.b) {
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                intake.setPower(0);
            }
            if (gamepad1.x) {
                intakeServo1.setPower(1.0);
                intakeServo2.setPower(-1.0);
            }
            else if (gamepad1.y) {
                intakeServo1.setPower(-1.0);
                intakeServo2.setPower(1.0);
            }
            intake.setPower(power);
            telemetry.addData("Power: ", power);
            telemetry.update();
        }

    }
}
