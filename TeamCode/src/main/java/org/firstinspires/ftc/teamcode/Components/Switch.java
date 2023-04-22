package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class Switch {

    DigitalChannel touchSensor;
    public static boolean prezzed = false;
    public Switch() {
        touchSensor = op.hardwareMap.get(DigitalChannel.class, "clawSwitch");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

        public boolean isSwitched() {
//        touchSensor.
            prezzed = touchSensor.getState();
        return prezzed;
//            return false;
        }

    }