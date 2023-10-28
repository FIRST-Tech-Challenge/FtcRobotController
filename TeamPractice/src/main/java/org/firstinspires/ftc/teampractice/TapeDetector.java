package org.firstinspires.ftc.teampractice;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class TapeDetector implements IObjectDetector<Boolean> {
    private ColorSensor sensorColor;

    public TapeDetector(ColorSensor colorSensor) {
        sensorColor = colorSensor;
    }

    @Override
    public Boolean objectDetected() {
        boolean detected = false;
        if (sensorColor.red() > 250){
            detected = true;
        }

        return detected;
    }
}
