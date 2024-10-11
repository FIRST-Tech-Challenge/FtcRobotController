package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestAnalogInput extends TestItem {
    private AnalogInput analogInput;
    private double min;
    private double max;

    public TestAnalogInput(String description, AnalogInput analogInput, double min, double max){
        super(description);
        this.analogInput = analogInput;
        this.min = min;
        this.max = max;
    }

    @Override
    public void run(boolean on, Telemetry telemetry){
        telemetry.addData("Voltage", analogInput.getVoltage());
        telemetry.addData("In Range:",
                Range.scale(analogInput.getVoltage(),
                0, analogInput.getMaxVoltage(),
                min, max));
    }
}
