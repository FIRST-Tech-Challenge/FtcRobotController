package org.firstinspires.ftc.teamcode.Old.Components.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class Switch {

    DigitalChannel touchSensor;

    public Switch() {
//        noMoSwitch
//        touchSensor = opMode.hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isSwitched() {

        return !touchSensor.getState();
    }

}
