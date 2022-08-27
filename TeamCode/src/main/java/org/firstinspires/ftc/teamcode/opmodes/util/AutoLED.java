package org.firstinspires.ftc.teamcode.opmodes.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;

import androidx.annotation.NonNull;

public class AutoLED {
    private final DigitalChannel redLED;
    private final DigitalChannel greenLED;
    private final Thread updateThread;

    public AutoLED(@NonNull HardwareMap hardwareMap, TseDetector detector) {
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        this.updateThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                switch (detector.run()) {
                    case 1:
                        redLED.setState(true);
                        greenLED.setState(false);
                        break;
                    case 2:
                        redLED.setState(true);
                        greenLED.setState(true);
                        break;
                    case 3:
                        redLED.setState(false);
                        greenLED.setState(true);
                        break;
                }
            }
        });
        
        this.updateThread.setPriority(Thread.NORM_PRIORITY);
        this.updateThread.start();
    }

    public void stop() {
        this.updateThread.interrupt();
        redLED.setState(false);
        greenLED.setState(false);
    }
}
