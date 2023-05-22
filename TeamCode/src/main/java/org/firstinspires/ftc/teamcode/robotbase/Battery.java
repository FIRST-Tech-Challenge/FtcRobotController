package org.firstinspires.ftc.teamcode.robotbase;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Battery {

    private final VoltageSensor sensor;

    public Battery(HardwareMap hardwareMap)
    {
        sensor = hardwareMap.voltageSensor.iterator().next();
    }

    public double getVoltage()
    {
        return sensor.getVoltage();
    }
}
