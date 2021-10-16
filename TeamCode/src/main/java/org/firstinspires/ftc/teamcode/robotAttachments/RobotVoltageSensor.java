package org.firstinspires.ftc.teamcode.robotAttachments;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RobotVoltageSensor {
    private VoltageSensor sensor;

    public RobotVoltageSensor(HardwareMap hardwareMap) {
        for (com.qualcomm.robotcore.hardware.VoltageSensor _sensor : hardwareMap.voltageSensor) {
            sensor = _sensor;
        }
    }

    public double getVoltage() {
        return sensor.getVoltage();
    }
}
