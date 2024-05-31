package org.firstinspires.ftc.teamcode.org.rustlib.core;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.org.rustlib.rustboard.Rustboard;
import org.firstinspires.ftc.teamcode.org.rustlib.utils.SuperGamepad;

import java.util.List;

public abstract class RobotBase extends OpMode {
    protected LynxModule controlHub;
    protected LynxModule expansionHub;
    protected SuperGamepad driveController;
    protected SuperGamepad payloadController;

    static {
        Rustboard.getInstance().start();
    }

    @Override
    public final void init() {
        CommandScheduler.getInstance().clearRegistry();
        CommandScheduler.getInstance().cancelAll();
        Rustboard.getInstance().start();
        Rustboard.getInstance().newLog();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        if (hubs.get(0).isParent()) {
            controlHub = hubs.get(0);
            expansionHub = hubs.get(1);
        } else {
            controlHub = hubs.get(1);
            expansionHub = hubs.get(0);
        }
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        driveController = new SuperGamepad(gamepad1);
        payloadController = new SuperGamepad(gamepad2);
        setup();
    }

    @Override
    public final void init_loop() {
        CommandScheduler.getInstance().run();
        setupLoop();
    }

    @Override
    public final void start() {
        if (this instanceof Auton) {
            Command autonomousCommand = ((Auton) this).getAutonomousCommand();
            autonomousCommand.schedule();
        }
        onStart();
    }

    @Override
    public final void loop() {
        CommandScheduler.getInstance().run();
        mainLoop();
    }


    @Override
    public final void stop() {
        CommandScheduler.getInstance().clearRegistry();
        onStop();
    }

    public void setup() {

    }

    public void setupLoop() {

    }

    public void onStart() {

    }

    public void mainLoop() {

    }

    public void onStop() {

    }
}
