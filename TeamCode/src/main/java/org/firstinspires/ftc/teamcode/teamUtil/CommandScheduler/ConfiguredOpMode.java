package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Scheduler;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX.GamepadEX;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

import java.util.ArrayList;

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
        Telemetry.Item initialising = telemetry.addData("", "");
        initialising.setCaption("Initialising");
        initialising.setValue("Robot");
        
        r = RobotConfig.freshInstance(this);
        gamepadEX1 = new GamepadEX(gamepad1);
        gamepadEX2 = new GamepadEX(gamepad2);
        r.setImu(r.opMode.hardwareMap.get(IMU.class, ConfigNames.imu));
        r.getImu().initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        r.resetIMU();
        superInit();
        
        Telemetry.Item initialisedSubsystems = telemetry.addData("","");
        initialisedSubsystems.setCaption("Initialised");
        StringBuilder initialisationSequencer = new StringBuilder("\nRobot");
        initialisedSubsystems.setValue(initialisationSequencer);
        
        for (int i = 0; i < r.subsystems.size(); i++) {
            Subsystem subsystem = r.subsystems.get(i);
            String[] subsystemStringSplit = subsystem.getClass().toString().split("\\.");
            String subsystemString = subsystemStringSplit[subsystemStringSplit.length-1];
            initialising.setValue(subsystemString);
            subsystem.init();
            initialisationSequencer.append("\n");
            initialisationSequencer.append(subsystemString);
            initialisedSubsystems.setValue(initialisationSequencer);
        }
        initialising.setValue("");
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
