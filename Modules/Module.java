package org.firstinspires.ftc.teamcode.rework.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Module {
    HardwareMap hardwareMap;

    public Module() {

    }

    public Module(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    protected DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);

        } catch (IllegalArgumentException exception) {
            return null;
        }
    }
}
