package org.firstinspires.ftc.teamcode.teamUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Assists with update loops and the like for the integration of robotConfig.
 * Grants access to the robotConfig instance at creation too.
 * To be used instead of OpMode
 */
public abstract class ConfiguredOpMode extends OpMode {

    public RobotConfig r;

    @Override
    public void init() {
        r = RobotConfig.getInstance(this);
        superInit();
    }
    public abstract void superInit();

    public abstract void superInit_Loop();
    @Override
    public void init_loop() {

        superInit_Loop();

    }

    public abstract void superStart();
    @Override
    public void start() {
        r.telemetry.clear();
        superStart();
    }

    public abstract void superLoop();
    @Override
    public void loop() {
        r.systemsStartLoopUpdate();
        superLoop();
        r.systemsEndLoopUpdate();
    }

    public abstract void superStop();
    @Override
    public void stop(){
        superStop();
        r.closeLogs();
        r.lockDown();
    }
}
