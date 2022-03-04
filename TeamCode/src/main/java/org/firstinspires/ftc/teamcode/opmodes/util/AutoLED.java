package org.firstinspires.ftc.teamcode.opmodes.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;

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
                        break;
                    case 2:
                        redLED.setState(true);
                    case 3:
                        greenLED.setState(true);
                        break;
                }
            }
        });
    }

    public void stop() {
        this.updateThread.interrupt();
    }
}
