package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.ButtonEX;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class LimitSwitch {
    RobotConfig r;

    private final TouchSensor limitSwitch;

    public final ButtonEX limitSwitchEX;

    public LimitSwitch(RobotConfig r) {
        this.r = r;

        limitSwitch = r.hardwareMap.get(TouchSensor.class, "limit");
        limitSwitchEX = new ButtonEX();
    }

    public void startLoopUpdate(){
        limitSwitchEX.startLoopUpdate(limitSwitch.isPressed());
    }

    public void endLoopUpdate(){
        limitSwitchEX.endLoopUpdate();
    }
}
