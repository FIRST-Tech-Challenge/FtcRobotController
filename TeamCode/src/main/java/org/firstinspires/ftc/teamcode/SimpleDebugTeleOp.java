package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp
public class SimpleDebugTeleOp extends OpMode {

    DcMotor motorTest;

    @Override
    public void init() {
      motorTest = hardwareMap.get(DcMotor.class, "motorTest");



    }

    @Override
    public void loop() {
        double power = 0;
        if (Math.abs(gamepad1.left_stick_y) > .1) {
            power = gamepad1.left_stick_y * .5;
        }

        motorTest.setPower(power);
    }
}
