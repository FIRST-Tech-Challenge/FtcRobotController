package org.firstinspires.ftc.teamcode.epicDemo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExternalTelemetryClass {
    public Telemetry telemetry;
    public ExternalTelemetryClass(Telemetry telemetryInput, HardwareMap hardwareMap){
        System.out.println("Something working");
        telemetry = telemetryInput;
    }

    public void kennansEpicFunction() {
        telemetry.addLine("Epic");
    }
}
