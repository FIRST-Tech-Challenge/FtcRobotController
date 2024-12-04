
package com.kalipsorobotics.modules;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RevLED {
    private DigitalChannel redLed;
    private DigitalChannel greenLed;
    private DigitalChannel redLed2;
    private DigitalChannel greenLed2;
    private DigitalChannel redLed3;
    private DigitalChannel greenLed3;
    private DigitalChannel redLed4;
    private DigitalChannel greenLed4;

    public RevLED(HardwareMap hardwareMap, String redPort, String greenPort, String redPort2, String greenPort2, String redPort3, String greenPort3, String redPort4, String greenPort4) {
        redLed = hardwareMap.get(DigitalChannel.class, redPort);
        greenLed = hardwareMap.get(DigitalChannel.class, greenPort);
        redLed2 = hardwareMap.get(DigitalChannel.class, redPort2);
        greenLed2 = hardwareMap.get(DigitalChannel.class, greenPort2);
        redLed3 = hardwareMap.get(DigitalChannel.class, redPort3);
        greenLed3 = hardwareMap.get(DigitalChannel.class, greenPort3);
        redLed4 = hardwareMap.get(DigitalChannel.class, redPort4);
        greenLed4 = hardwareMap.get(DigitalChannel.class, greenPort4);

        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);
        redLed2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed2.setMode(DigitalChannel.Mode.OUTPUT);
        redLed3.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed3.setMode(DigitalChannel.Mode.OUTPUT);
        redLed4.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed4.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void turnOnRed() {
        redLed.setState(false);
        redLed2.setState(false);
        redLed3.setState(false);
        redLed4.setState(false);
        greenLed.setState(true);
        greenLed2.setState(true);
        greenLed3.setState(true);
        greenLed4.setState(true);
    }

    public void turnOnGreen() {
        redLed.setState(true);
        redLed2.setState(true);
        redLed3.setState(true);
        redLed4.setState(true);
        greenLed.setState(false);
        greenLed2.setState(false);
        greenLed3.setState(false);
        greenLed4.setState(false);
    }

    public void turnoff() {
        redLed.setState(true);
        redLed2.setState(true);
        redLed3.setState(true);
        redLed4.setState(true);
        greenLed.setState(true);
        greenLed2.setState(true);
        greenLed3.setState(true);
        greenLed4.setState(true);
    }

}

