package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;



public class RangeSensor {

    private AnalogInput ultrasonic;

    public RangeSensor(LinearOpMode opMode) {
        ultrasonic =  opMode.hardwareMap.get(AnalogInput.class, "ultrasonic");
    }

    public double getDistance() {
        double rawValue = ultrasonic.getVoltage();
        double voltage_scale_factor = (ultrasonic.getVoltage() / ((3.3/1024.0) * 6.0)) - 300.0;
        return rawValue * voltage_scale_factor * 0.0492;
    }
}