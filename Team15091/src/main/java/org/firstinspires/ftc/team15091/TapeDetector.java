package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.team15091.IObjectDetector;

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
