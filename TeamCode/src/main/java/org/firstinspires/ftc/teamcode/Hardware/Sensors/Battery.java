package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class Battery {
    public static double threshold = 0.5;
    HardwareMap hardwareMap;
    public VoltageSensor voltageSensor;
    public ElapsedTime timer = new ElapsedTime();
    double voltage;
    public Battery (HardwareMap hwmap){
        this.hardwareMap = hwmap;
        this.voltageSensor = hwmap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
    }


    public double getVoltage() {
        double timeLastUpdate = timer.seconds();
        if (timeLastUpdate > threshold) {
            voltage = voltageSensor.getVoltage();
            timer.reset();
        }
        return voltage;
    }
}
