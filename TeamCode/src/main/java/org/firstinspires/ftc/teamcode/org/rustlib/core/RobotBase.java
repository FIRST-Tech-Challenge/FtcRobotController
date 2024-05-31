package org.firstinspires.ftc.teamcode.org.rustlib.core;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.org.rustlib.rustboard.Rustboard;
import org.firstinspires.ftc.teamcode.org.rustlib.utils.SuperGamepad;

import java.util.ArrayList;
import java.util.List;

public abstract class RobotBase extends OpMode {
    public enum Alliance {
        BLUE,
        RED
    }

    public enum GameElementLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    public static Alliance alliance = Alliance.BLUE;
    protected LynxModule controlHub;
    protected LynxModule expansionHub;
    protected SuperGamepad driveController;
    protected SuperGamepad payloadController;
    public static final ArrayList<Runnable> onInitCallbacks = new ArrayList<>(); // TODO: make these private and add methods to add callbacks.  Otherwise whenever init is called duplicate callbacks may be added
    public static final ArrayList<Runnable> onStartCallbacks = new ArrayList<>();
    public static final ArrayList<Runnable> onStopCallbacks = new ArrayList<>();

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
        runCallbacks(onInitCallbacks);
        onInit();
    }

    @Override
    public final void init_loop() {
        CommandScheduler.getInstance().run();
        initLoop();
    }

    @Override
    public final void start() {
        if (this instanceof Auton) {
            Command autonomousCommand = ((Auton) this).getAutonomousCommand();
            autonomousCommand.schedule();
        }
        runCallbacks(onStartCallbacks);
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
        CommandScheduler.getInstance().cancelAll();
        runCallbacks(onStopCallbacks);
        onStop();
    }

    private void runCallbacks(ArrayList<Runnable> callbacks) {
        for (Runnable callback : callbacks) {
            callback.run();
        }
    }

    public void onInit() {

    }

    public void initLoop() {

    }

    public void onStart() {

    }

    public void mainLoop() {

    }

    public void onStop() {

    }
}
