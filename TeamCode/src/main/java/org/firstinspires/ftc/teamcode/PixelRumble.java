package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PixelRumble {
    final int MAX_TICKS = 100;

    Controller controller;

    int ticksRemaining = 0;
    //double tempSpeed = 0;

    public PixelRumble(Controller controller, HardwareMap hardwareMap) {
        this.controller = controller;
    }

    public void Rumble(Gamepad gamepad1, Gamepad gamepad2) {
        RumbleSpeed(gamepad1, gamepad2, 0.1);
//        tempSpeed -= gamepad2.right_stick_y / 1000;
//        RumbleSpeed(gamepad1, gamepad2, tempSpeed);
//        controller.Debug("temp speed", tempSpeed);
    }

    public void RumbleSpeed(Gamepad gamepad1, Gamepad gamepad2, double speed) {
        if (speed > 1 || speed < 0) return;

        if (ticksRemaining == 0) {
            gamepad1.rumble(100);
            gamepad2.rumble(100);
            ticksRemaining = MAX_TICKS - (int)(speed * MAX_TICKS);
        }
        else {
            ticksRemaining--;
        }
    }
}
