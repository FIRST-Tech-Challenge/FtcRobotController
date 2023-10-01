package org.firstinspires.ftc.teampractice.examples;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teampractice.IObjectDetector;

public class TapeDetector implements IObjectDetector<Boolean> {
    private ColorSensor sensorColor;

    public TapeDetector(ColorSensor colorSensor) {
        sensorColor = colorSensor;
    }

    @Override
    public Boolean objectDetected() {
        boolean detected = false;
        if (sensorColor.blue() > 250 || sensorColor.red() > 250) {
            detected = true;
        }

        return detected;
    }
}
