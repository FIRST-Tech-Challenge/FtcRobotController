package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SpinnerHardware;

@TeleOp(name="Spinner Test")
public class TestSpinner extends SpinnerHardware {
//spinny test spinner lmao
    @Override
    public void loop() {
        spinnerLeft.setPower(gamepad1.right_stick_y);
        spinnerRight.setPower((-gamepad1.right_stick_y));
    }
}
