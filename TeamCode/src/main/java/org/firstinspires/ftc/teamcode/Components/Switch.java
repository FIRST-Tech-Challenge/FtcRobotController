package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class Switch {

    DigitalChannel touchSensor;
    public Switch() {
        touchSensor = op.hardwareMap.get(DigitalChannel.class, "clawSwitch");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
//        touchSensor.resetDeviceConfigurationForOpMode();
//        touchSensor.
//        touchSensor.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public boolean isSwitched() {
//        touchSensor.
        return touchSensor.getState();
    }

}
