package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.Map;

//i still have no clue what the intake needs lmao

public class Intake implements Subsystem {
    //CONSTANTS
    HardwareMap hardwareMap;
    Robot_fromScratch robot;
    //TODO - ACTUALLY FIGURE THIS OUT
    public static double GRIP_TICKS = 500;
    public static double RELEASE_TICKS = 800;

    public enum Articulation {
        MANUAL,
        GRAB_PIXEL,
        FOLD,
    }


    //LIVE STATES
    public static boolean gripped;
    public static double clawTicks;
    public Articulation articulation;


    public Intake(HardwareMap hardwareMap, Robot_fromScratch robot) {
            this.hardwareMap = hardwareMap;
            this.robot = robot;
            articulation = Articulation.FOLD;
    }

    @Override
    public void update(Canvas fieldOverlay) {

    }

    public Articulation articulate(Articulation target) {
        articulation = target;
        return articulation;
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("claw gripped?", gripped);
        telemetryMap.put("claw position", clawTicks);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "INTAKE";
    }
}
