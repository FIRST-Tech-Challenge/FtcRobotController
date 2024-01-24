package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorControl", group = "Tests")
@Disabled
public class MotorControl extends LinearOpMode {
    DcMotorEx motor1, motor2, motor3, motor4;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motor2 = hardwareMap.get(DcMotorEx.class, "frontRight");
        motor3 = hardwareMap.get(DcMotorEx.class, "rearLeft");
        motor4 = hardwareMap.get(DcMotorEx.class, "rearRight");

        waitForStart();

        while(opModeIsActive()) {
            motor1.setPower(gamepad1.left_trigger);
            motor2.setPower(gamepad1.right_trigger);
            motor3.setPower(gamepad1.left_stick_y);
            motor4.setPower(gamepad1.right_stick_y);
        }
    }
}
