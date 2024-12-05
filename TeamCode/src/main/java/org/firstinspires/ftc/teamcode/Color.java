package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class Color {
    private ColorSensor colorSensor;
    private Telemetry telemetry;
    public Color(HardwareMap hardwareMap, Telemetry telemetryGet) {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry = telemetryGet;
    }
    public int colorSenseRed() {
        return colorSensor.red();
    }
    public int colorSenseGreen() {
        return colorSensor.green();
    }
    public int colorSenseBlue() {
        return colorSensor.blue();
    }
    public int senseObject() {
        return colorSensor.alpha();
    }
}