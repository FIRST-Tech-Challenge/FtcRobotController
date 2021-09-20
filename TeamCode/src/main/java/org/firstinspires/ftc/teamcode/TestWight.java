package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.io.File;

@TeleOp
public class TestWight  extends OpMode {
    private DcMotorEx motor;

    File out;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "Motor");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("position", motor::getCurrentPosition);
        telemetry.addData("velocity", motor::getVelocity);
        telemetry.update();
    }

    @Override
    public void loop() {
        motor.setPower(-gamepad1.left_stick_y);
        telemetry.update();
    }
}
