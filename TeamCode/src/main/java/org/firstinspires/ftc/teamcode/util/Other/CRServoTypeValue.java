package org.firstinspires.ftc.teamcode.util.Other;

import com.qualcomm.robotcore.hardware.CRServo;

public class CRServoTypeValue implements DynamicTypeValue<CRServo> {
    private CRServo value;

    public CRServoTypeValue(CRServo value) {
        this.value = value;
    }

    @Override
    public CRServo getValue() {
        return value;
    }

    @Override
    public void setValue(CRServo value) {
        this.value = value;
    }
}
