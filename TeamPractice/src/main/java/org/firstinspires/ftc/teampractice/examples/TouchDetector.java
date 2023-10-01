package org.firstinspires.ftc.teampractice.examples;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teampractice.IObjectDetector;

public class TouchDetector implements IObjectDetector<Boolean> {
    TouchSensor digitalTouch;  // Hardware Device Object

    public TouchDetector(TouchSensor sensorToUse) {
        digitalTouch = sensorToUse;
    }
    @Override
    public Boolean objectDetected() {
        return digitalTouch.isPressed();
    }

    public boolean isPressed() {
        return digitalTouch.isPressed();
    }
}
