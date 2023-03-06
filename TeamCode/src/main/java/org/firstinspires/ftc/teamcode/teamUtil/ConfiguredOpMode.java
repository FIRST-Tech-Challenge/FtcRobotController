package org.firstinspires.ftc.teamcode.teamUtil;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX.GamepadEX;

/**
 * Assists with update loops and the like for the integration of robotConfig.
 * Grants access to the robotConfig instance at creation too.
 * To be used instead of OpMode
 */
public abstract class ConfiguredOpMode extends OpMode {

    public RobotConfig r;
    public GamepadEX
        gamepadEX1,
        gamepadEX2;

    public abstract void superInit();
    public abstract void registerTriggers();
    @Override
    public void init() {
        r = RobotConfig.freshInstance(this);
        gamepadEX1 = new GamepadEX(gamepad1);
        gamepadEX2 = new GamepadEX(gamepad2);
        superInit();
        for (Subsystem subsystem : r.subsystems) {
            subsystem.init();
        }
        registerTriggers();
    }

    public abstract void superInit_Loop();
    @Override
    public void init_loop() {
        for (LynxModule module : r.allHubs) {
            module.clearBulkCache();
        }
        superInit_Loop();
    }

    public abstract void superStart();
    @Override
    public void start() {
        r.opMode.telemetry.clear();
        RobotConfig.elapsedTime.reset();
        superStart();
    }

    public abstract void superLoop();
    @Override
    public void loop() {
        for (LynxModule module : r.allHubs) {
            module.clearBulkCache();
        }
        for (Subsystem subsystem : r.subsystems) {
            subsystem.read();
        }
        gamepadEX1.startLoopUpdate();
        gamepadEX2.startLoopUpdate();
        superLoop();
        for (Subsystem subsystem : r.subsystems) {
            subsystem.update();
        }
        gamepadEX1.endLoopUpdate();
        gamepadEX2.endLoopUpdate();
    }

    public abstract void superStop();
    @Override
    public void stop(){
        superStop();
        for (Subsystem subsystem : r.subsystems) {
            subsystem.close();
        }
    }
}
