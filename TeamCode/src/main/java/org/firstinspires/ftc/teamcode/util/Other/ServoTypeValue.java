package org.firstinspires.ftc.teamcode.util.Other;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoTypeValue implements DynamicTypeValue <Servo> {
    private Servo value;

    public ServoTypeValue(Servo value) {
        this.value = value;
    }

    @Override
    public Servo getValue() {
        return this.value;
    }

    @Override
    public void setValue(Servo value) {
        this.value = value;
    }
}
