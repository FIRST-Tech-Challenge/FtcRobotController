package org.firstinspires.ftc.teamcode.Battery;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Battery {
    HardwareMap hardwareMap;
    VoltageSensor voltageSensor;
    public Battery (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Voltage Sensor");
    }

    public double getVoltage(){
        double volts = 0;

            double voltage = voltageSensor.getVoltage();
            if (voltage > 0) {
                volts = Math.min(volts, voltage);
            }
        return volts;
    }
}
