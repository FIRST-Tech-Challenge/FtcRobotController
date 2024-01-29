package org.firstinspires.ftc.teamcode.util.Other;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorTypeValue implements DynamicTypeValue<ColorSensor> {
    private ColorSensor value;

    public ColorSensorTypeValue(ColorSensor value) {
        this.value = value;
    }

    @Override
    public ColorSensor getValue() {
        return value;
    }

    @Override
    public void setValue(ColorSensor value) {
        this.value = value;
    }
}
