
package com.kalipsorobotics.modules;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RevLED {
    private DigitalChannel redLed;
    private DigitalChannel greenLed;

    public RevLED(HardwareMap hardwareMap, String redPort, String greenPort) {
        redLed = hardwareMap.get(DigitalChannel.class, redPort);
        greenLed = hardwareMap.get(DigitalChannel.class, greenPort);

        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void turnOnRed() {
        redLed.setState(false);
        greenLed.setState(true);
    }

    public void turnOnGreen() {
        redLed.setState(true);
        greenLed.setState(false);
    }

    public void turnoff() {
        redLed.setState(true);
        greenLed.setState(true);
    }

    }

