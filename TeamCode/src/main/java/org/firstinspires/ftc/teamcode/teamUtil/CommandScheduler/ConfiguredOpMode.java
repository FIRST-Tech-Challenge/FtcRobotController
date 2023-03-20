package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Scheduler;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX.GamepadEX;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

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
    public final void init() {
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
    public final void init_loop() {
        for (LynxModule module : r.allHubs) {
            module.clearBulkCache();
        }
        r.scheduler.pollSubsystems();
        superInit_Loop();
        r.scheduler.updateSubsystems();
    }

    public abstract void superStart();
    @Override
    public final void start() {
        r.opMode.telemetry.clear();
        RobotConfig.elapsedTime.reset();
        superStart();
    }

    public abstract void superLoop();
    @Override
    public final void loop() {
        for (LynxModule module : r.allHubs) {
            module.clearBulkCache();
        }
        r.scheduler.pollSubsystems();
        r.scheduler.pollTriggers();
        superLoop();
        r.scheduler.updateSubsystems();
        gamepadEX1.endLoopUpdate();
        gamepadEX2.endLoopUpdate();
    }

    public abstract void superStop();
    @Override
    public final void stop(){
        superStop();
        for (Subsystem subsystem : r.subsystems) {
            subsystem.close();
        }
    }
}
