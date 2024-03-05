package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class DrivingAssist {
    final int MAX_TICKS = 100;

    DigitalChannel leftSwitch;
    DigitalChannel rightSwitch;
    DigitalChannel led1;
    DigitalChannel led2;

    Controller controller;
    DetectPixel detectPixel;

    int ticksRemaining = 0;
    //double tempSpeed = 0;

    public DrivingAssist(Controller controller, HardwareMap hardwareMap, DetectPixel detectPixel) {
        this.controller = controller;
        this.detectPixel = detectPixel;

        leftSwitch = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        rightSwitch = hardwareMap.get(DigitalChannel.class, "rightSwitch");
        led1 = hardwareMap.get(DigitalChannel.class, "led1");
        led2 = hardwareMap.get(DigitalChannel.class, "led2");
        led1.setMode(DigitalChannel.Mode.OUTPUT);
        led2.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void gripLed() {
        led1.setState(!leftSwitch.getState());
        led2.setState(!rightSwitch.getState());
    }

    public void rumble(Gamepad gamepad1, Gamepad gamepad2) {
        rumbleSpeed(gamepad1, gamepad2, 0.1);
        //detectPixel.telemetryTfod();
        controller.Debug("test y", detectPixel.getDistance());

//        tempSpeed -= gamepad2.right_stick_y / 1000;
//        RumbleSpeed(gamepad1, gamepad2, tempSpeed);
//        controller.Debug("temp speed", tempSpeed);
    }

    void rumbleSpeed(Gamepad gamepad1, Gamepad gamepad2, double speed) {
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
