package org.firstinspires.ftc.teamcode.util.Other;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorTypeValue implements DynamicTypeValue<DcMotor> {
    private DcMotor value;

    public MotorTypeValue(DcMotor value) {
        this.value = value;
    }

    @Override
    public DcMotor getValue() {
        return value;
    }

    @Override
    public void setValue(final DcMotor value) {
        this.value = value;
    }
}
