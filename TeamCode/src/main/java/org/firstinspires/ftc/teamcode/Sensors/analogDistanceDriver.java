package org.firstinspires.ftc.teamcode.hardware.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class analogDistanceDriver{

    private final double maxR;
    private AnalogInput analog;
    private double voltage;

    public analogDistanceDriver(AnalogInput analog, double maxR) {
        this.analog = analog;
        this.maxR = maxR;
        this.voltage = getVoltage();
    }


    public double getVoltage() {
        voltage = this.analog.getVoltage();
        return voltage;
    }

    public double update() {
        double distance = (voltage*this.maxR)/3.3;
        return distance;
    }
}


