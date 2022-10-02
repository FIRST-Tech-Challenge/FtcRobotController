package org.firstinspires.ftc.teamcode.robots.LilVirani.subsystem;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.craneIK;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * @author Mahesh Natamai
 */

@Config(value = "FFRobot")
public class Robot implements Subsystem {

    public DriveTrain driveTrain;
    public Turret turret;
    public Crane crane;
    public Subsystem[] subsystems;

    private long[] subsystemUpdateTimes;
    private boolean autoDumpEnabled, doubleDuckEnabled;

    private final List<LynxModule> hubs;

    private VoltageSensor batteryVoltageSensor;

    private Articulation articulation;
    private final Map<Articulation, StateMachine> articulationMap;

    private Bitmap craneBitmap;
    private Mat craneMat;
    public static int CB_WIDTH = 320;
    public static int CB_HEIGHT = 240;

    public Robot(HardwareMap hardwareMap, boolean simulated) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap);
        turret = new Turret(hardwareMap, simulated);
        crane = new Crane(hardwareMap, turret, simulated);

        subsystems = new Subsystem[] {driveTrain, crane};
        subsystemUpdateTimes = new long[subsystems.length];

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        articulation = Articulation.MANUAL;

        articulationMap = new HashMap<>();

        craneBitmap = Bitmap.createBitmap(CB_WIDTH, CB_HEIGHT, Bitmap.Config.RGB_565);
        craneMat = new Mat(CB_HEIGHT, CB_WIDTH, CvType.CV_8UC3);
    }

    public void handleTankDrive(Gamepad a, Gamepad b){
        driveTrain.leftMotor.setPower(a.left_stick_y);
        driveTrain.leftMotor.setPower(a.right_stick_y);
    }

    public void handleGridDrive(Gamepad a, Gamepad b){

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Articulation", articulation);
        telemetryMap.put("auto-dump enabled", autoDumpEnabled);
        if(debug) {
            for (int i = 0; i < subsystems.length; i++) {
                String name = subsystems[i].getClass().getSimpleName();
                telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
            }
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Robot";
    }

    @Override
    public void update(Canvas fieldOverlay) {
        for (LynxModule module : hubs)
            module.clearBulkCache();

    }

    public Bitmap getBitmap() {
        return craneBitmap;
    }

    @Override
    public void stop() {
        for(Subsystem subsystem: subsystems)
            subsystem.stop();
    }

    public double getVoltage(){return batteryVoltageSensor.getVoltage();}

    //----------------------------------------------------------------------------------------------
    // Articulations
    //----------------------------------------------------------------------------------------------

    public enum Articulation {
        MANUAL,

        // misc. articulations
        INIT,
        START,
        START_DOWN, // use to prep for start - stows the crane
        START_END_GAME, //use on a timer to automatically deploy carousel spinner 10 seconds before end game

        // tele-op articulations
        TRANSFER,
        DUMP_AND_SET_CRANE_FOR_TRANSFER,
        GRAB_AND_TRANSFER,

        AUTO_HIGH_TIER_RED,
        AUTO_HIGH_TIER_BLUE,
        AUTO_MIDDLE_TIER_RED,
        AUTO_MIDDLE_TIER_BLUE,
        AUTO_LOW_TIER_RED,
        AUTO_LOW_TIER_BLUE,

        DOUBLE_DUCK_GRAB_AND_TRANSFER,
        DOUBLE_DUCK_DUMP_AND_SET_CRANE_FOR_TRANSFER
    }

    public boolean articulate(Articulation articulation) {
        if(articulation.equals(Articulation.MANUAL))
            return true;
        this.articulation = articulation;
        if(articulationMap.get(articulation).execute()) {
            this.articulation = Articulation.MANUAL;
            return true;
        }
        return false;
    }
}
