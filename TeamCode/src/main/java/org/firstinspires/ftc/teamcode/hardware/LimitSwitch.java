package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class LimitSwitch {

    private TouchSensor limitSwitch;
    private boolean isPressed;

    public LimitSwitch(HardwareMap hwMap, String id) {
        limitSwitch = hwMap.get(TouchSensor.class, id);
    }

    public boolean isActivated() {
        return this.isPressed;
    }

    public void update() {
        this.isPressed = this.limitSwitch.isPressed();
    }
}
