package org.rustlib.core;

import android.util.Pair;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.rustlib.commandsystem.Command;
import org.rustlib.commandsystem.CommandScheduler;
import org.rustlib.hardware.SuperGamepad;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public abstract class RobotBase extends OpMode {
    private static final ArrayList<ArrayList<Pair<OpModeState, Runnable>>> callbacks = new ArrayList<>();
    public static Alliance alliance = Alliance.BLUE;
    static WeakReference<RobotControllerActivity> mainActivity;
    private static OpModeState opModeState = OpModeState.IDLE;
    protected LynxModule controlHub = null;
    protected LynxModule expansionHub = null;
    protected SuperGamepad controller1;
    protected SuperGamepad controller2;
    private Auton autonomousInstance = null;
    private OpModeCore opModeCoreInstance = null;

    public RobotBase() {
        for (int i = 0; i < 3; i++) {
            callbacks.add(new ArrayList<>());
        }
        if (this instanceof Auton) {
            autonomousInstance = (Auton) this;
        }
        if (this instanceof OpModeCore) {
            opModeCoreInstance = (OpModeCore) this;
        }
    }

    private static void addCallback(Runnable callback, Collection<Pair<OpModeState, Runnable>> targetCollection) {
        targetCollection.add(new Pair<>(opModeState, callback));
    }

    public static final void onOpModeInit(Runnable callback) {
        addCallback(callback, callbacks.get(0));
    }

    public static final void onOpModeStart(Runnable callback) {
        addCallback(callback, callbacks.get(1));
    }

    public static final void onOpModeStop(Runnable callback) {
        addCallback(callback, callbacks.get(2));
    }

    public static RobotControllerActivity getMainActivity() {
        return mainActivity.get();
    }

    private void removeDuplicateCallbacks() {
        for (Collection<Pair<OpModeState, Runnable>> targetCollection : callbacks) {
            for (Pair<OpModeState, Runnable> callback : targetCollection) {
                if (callback.first == opModeState) {
                    targetCollection.remove(callback);
                }
            }
        }
    }

    public OpModeState getOpModeState() {
        return opModeState;
    }

    private static void setBulkCachingOn(LynxModule hub) {
        if (hub != null)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    @Override
    public final void init() {
        opModeState = OpModeState.INIT;
        removeDuplicateCallbacks();
        CommandScheduler.getInstance().clearRegistry();
        CommandScheduler.getInstance().cancelAll();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        if (hubs.get(0).isParent()) {
            controlHub = hubs.get(0);
            if (hubs.size() > 1) {
                expansionHub = hubs.get(1);
            }
        } else {
            controlHub = hubs.get(1);
            if (hubs.size() > 1) {
                expansionHub = hubs.get(0);
            }
        }
        setBulkCachingOn(controlHub);
        setBulkCachingOn(expansionHub);
        controller1 = new SuperGamepad(gamepad1);
        controller2 = new SuperGamepad(gamepad2);
        runCallbacks(callbacks.get(0));
        robotInit();
        if (opModeCoreInstance != null) {
            opModeCoreInstance.opModeInit();
        }
    }

    @Override
    public final void init_loop() {
        opModeState = OpModeState.INIT_LOOP;
        CommandScheduler.getInstance().run();
        robotInitLoop();
        if (opModeCoreInstance != null) {
            opModeCoreInstance.opModeInitLoop();
        }
    }

    @Override
    public final void start() {
        opModeState = OpModeState.START;
        removeDuplicateCallbacks();
        if (autonomousInstance != null) {
            Command autonomousCommand = autonomousInstance.getAutonomousCommand();
            if (autonomousCommand != null)
                autonomousCommand.schedule();
        }
        runCallbacks(callbacks.get(1));
        robotStart();
        if (opModeCoreInstance != null) {
            opModeCoreInstance.opmodeStart();
        }
    }

    @Override
    public final void loop() {
        opModeState = OpModeState.LOOP;
        CommandScheduler.getInstance().run();
        robotLoop();
        if (opModeCoreInstance != null) {
            opModeCoreInstance.opModeLoop();
        }
    }

    @Override
    public final void stop() {
        opModeState = OpModeState.STOP;
        removeDuplicateCallbacks();
        CommandScheduler.getInstance().clearRegistry();
        CommandScheduler.getInstance().cancelAll();
        runCallbacks(callbacks.get(2));
        robotStop();
        if (opModeCoreInstance != null) {
            opModeCoreInstance.opModeStop();
        }
        opModeState = OpModeState.IDLE;
    }

    private void runCallbacks(Collection<Pair<OpModeState, Runnable>> callbacks) {
        for (Pair<OpModeState, Runnable> callback : callbacks) {
            callback.second.run();
        }
    }

    public void robotInit() {

    }

    public void robotInitLoop() {

    }

    public void robotStart() {

    }

    public void robotLoop() {

    }

    public void robotStop() {

    }

    public enum Alliance {
        BLUE,
        RED
    }

    public enum GameElementLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum OpModeState {
        IDLE,
        INIT,
        INIT_LOOP,
        START,
        LOOP,
        STOP,
    }
}
