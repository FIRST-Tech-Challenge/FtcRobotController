package org.firstinspires.ftc.teamcode.util.Other;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class DistanceSensorTypeValue implements DynamicTypeValue<DistanceSensor> {
    private DistanceSensor value;

    public DistanceSensorTypeValue(DistanceSensor value) {
        this.value = value;
    }

    @Override
    public DistanceSensor getValue() {
        return value;
    }

    @Override
    public void setValue(DistanceSensor value) {
        this.value = value;
    }
}
