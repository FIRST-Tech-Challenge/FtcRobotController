package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX.ButtonEX;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class LimitSwitch extends Subsystem {
    RobotConfig r;

    private TouchSensor limitSwitch;

    public ButtonEX limitSwitchEX;

    public LimitSwitch(RobotConfig r) {
        this.r = r;
    }
    public LimitSwitch(){
        r = RobotConfig.getInstance();
    }

    @Override
    public void init() {
        limitSwitch = r.opMode.hardwareMap.get(TouchSensor.class, "limit");
        limitSwitchEX = new ButtonEX();
    }

    @Override
    public void read() {
        limitSwitchEX.startLoopUpdate(limitSwitch.isPressed());
    }

    @Override
    public void update() {
        limitSwitchEX.endLoopUpdate();
    }

    @Override
    public void close() {

    }
}
