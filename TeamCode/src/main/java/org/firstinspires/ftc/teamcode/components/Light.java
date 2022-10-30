package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Light {

    private DigitalChannel led;

    public Light(DigitalChannel led) {
        this.led = led;
        led.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void on() {
        led.setState(true);
    }

    public void off() {
        led.setState(false);
    }
}