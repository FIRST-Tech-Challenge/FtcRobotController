package org.firstinspires.ftc.teamcode.rework.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Module {
    HardwareMap hardwareMap;

    public Module() {

    }

    public Module(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    abstract public void init();

    abstract public void update();

    protected DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);

        } catch (IllegalArgumentException exception) {
            return null;
        }
    }
}
