package com.kalipsorobotics.modules;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RevLED {
    private static DigitalChannel redLed;
    private static DigitalChannel greenLed;

    public RevLED(HardwareMap hardwareMap, String redPort, String greenPort) {
        redLed = hardwareMap.get(DigitalChannel.class, redPort);
        greenLed = hardwareMap.get(DigitalChannel.class, greenPort);

        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public static void turnOnRed() {
        redLed.setState(false);
        greenLed.setState(true);
    }

    public static void turnOnGreen() {
        greenLed.setState(false);
        redLed.setState(true);
    }

}
