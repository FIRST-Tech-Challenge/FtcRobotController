//
// Created by Neil Rodriguez 12/07/2021
//

package org.firstinspires.ftc.teamcode.Core.BaseClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Module;

public abstract class EctoSensor extends Module {


    public EctoSensor(String moduleName, String moduleType) {
        super(moduleName, moduleType);
    }

    public void initSensor(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    //Every mechanism should be able to do this functions
    public abstract void initSensor();
    public abstract void startSensor();
    public abstract void updateSensor();
    public abstract void stopSensor();
}