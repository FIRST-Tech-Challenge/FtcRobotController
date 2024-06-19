package org.firstinspires.ftc.masters.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

abstract public class Component {
    protected HardwareMap hardwareMap = null;
    abstract void initializeHardware();
}
