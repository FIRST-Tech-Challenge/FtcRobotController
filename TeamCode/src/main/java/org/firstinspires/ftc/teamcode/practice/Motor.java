package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp
public class Motor extends OpMode {

    DcMotor motor;
    DcMotor motor2;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "test motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            motor.setPower(0.5);
            motor2.setPower(0.5);
        } else if (gamepad1.b) {
            motor.setPower(-0.5);
            motor2.setPower(0.5);
        } else {
            motor.setPower(0);
            motor2.setPower(0);
        }
    }
}



