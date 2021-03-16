package org.wheelerschool.robotics.comp.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wheelerschool.robotics.comp.CompBot;
import org.wheelerschool.robotics.lib.StatefulButton;

@TeleOp
public class AdjustDrive extends OpMode {
    CompBot hw;

    StatefulButton adjustUp;
    StatefulButton adjustDown;

    float powerFactor = 1.f;

    @Override
    public void init() {
        hw = new CompBot(hardwareMap);

        adjustUp = new StatefulButton(false);
        adjustDown = new StatefulButton(false);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if(adjustUp.update(gamepad1.dpad_up)) {
            powerFactor += 0.1f;
        } else if(adjustDown.update(gamepad1.dpad_down)) {
            powerFactor -= 0.1f;
        }

        telemetry.addData("Power factor", powerFactor);

        hw.launchLeft.setPower(powerFactor);
        hw.launchRight.setPower(1);

        hw.launchPush(gamepad1.right_bumper);
    }

    @Override
    public void stop() {
        hw.stop();
    }
}
