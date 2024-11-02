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

        double inputSpeed = -gamepad1.left_stick_y;

        // put some "slop" in, in case joystick is too sensitive
        if (Math.abs(inputSpeed) < .1) {
            inputSpeed = 0;
        }

        if (gamepad1.a) {
            power = inputSpeed * .5;
        }

        motorTest.setPower(power);
    }
}
