package org.firstinspires.ftc.teamcode.Battery;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class Battery {
    HardwareMap hardwareMap;
    VoltageSensor voltageSensor;
    public ElapsedTime timer = new ElapsedTime();
    public Battery (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Voltage Sensor");
    }

    public double getVoltage() {
        double voltage = voltageSensor.getVoltage();
        double timeLastUpdate = timer.seconds();
        if (timeLastUpdate > 0.5) {
            voltage = voltageSensor.getVoltage();
            timer.reset();
        }
        return voltage;
    }
}
