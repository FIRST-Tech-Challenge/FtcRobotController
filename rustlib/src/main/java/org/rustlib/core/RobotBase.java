package org.rustlib.core;

import android.util.Pair;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.rustlib.commandsystem.Command;
import org.rustlib.commandsystem.CommandScheduler;
import org.rustlib.drive.DriveSubsystem;
import org.rustlib.geometry.Rotation2d;
import org.rustlib.hardware.SuperGamepad;

import java.lang.annotation.Annotation;
import java.lang.ref.WeakReference;
import java.lang.reflect.Field;
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
    private OpModeCore opModeCoreInstance = null;
    private static DriveSubsystem autonomousDriveSubsystem;
    private static DriveSubsystem teleOpDriveSubsystem;

    private enum OpModeType {
        AUTO,
        TELE
    }

    private OpModeType opModeType = OpModeType.AUTO;

    public RobotBase() {
        for (int i = 0; i < 3; i++) {
            callbacks.add(new ArrayList<>());
        }
        if (this instanceof OpModeCore) {
            opModeCoreInstance = (OpModeCore) this;
        }
        if (hasAnnotation(Autonomous.class)) {
            autonomousDriveSubsystem = getDriveSubsystem(this.getClass()); // getClass() returns the runtime class of the instance in question
        } else if (hasAnnotation(TeleOp.class)) {
            teleOpDriveSubsystem = getDriveSubsystem(this.getClass());
            opModeType = OpModeType.TELE;
        }
    }

    private boolean hasAnnotation(Class<?> annotationType) {
        for (Annotation annotation : this.getClass().getAnnotations()) {
            if (annotation.annotationType().equals(annotationType)) {
                return true;
            }
        }
        return false;
    }

    private DriveSubsystem getDriveSubsystem(Class<?> opModeClass) {
        DriveSubsystem driveSubsystem = null;
        for (Field field : opModeClass.getDeclaredFields()) {
            if (field.getType().equals(DriveSubsystem.class)) {
                try {
                    driveSubsystem = (DriveSubsystem) field.get(this);
                } catch (IllegalAccessException e) {
                    driveSubsystem = null;
                }
            }
        }
        return driveSubsystem;
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

    public static boolean opModeRunning() {
        return RobotControllerActivity.opModeRunning();
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
            expansionHub = hubs.get(0);
            controlHub = hubs.get(1);
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
        if (opModeType == OpModeType.TELE && teleOpDriveSubsystem != null) {
            teleOpDriveSubsystem.setFieldCentricOffset(alliance == Alliance.BLUE ? new Rotation2d() : new Rotation2d(Math.PI));
        }
        removeDuplicateCallbacks();
        runCallbacks(callbacks.get(1));
        robotStart();
        if (opModeCoreInstance != null) {
            opModeCoreInstance.opmodeStart();
        }
        if (this instanceof AutonomousCore) {
            Command autonomousCommand = ((AutonomousCore) this).getAutonomousCommand();
            if (autonomousCommand != null)
                autonomousCommand.schedule();
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
        if (opModeType == OpModeType.AUTO && autonomousDriveSubsystem != null && teleOpDriveSubsystem != null) {
            teleOpDriveSubsystem.getOdometry().setPosition(autonomousDriveSubsystem.getOdometry().getPosition());
        }
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
