package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.Map;

public class Outtake implements Subsystem {

    HardwareMap hardwareMap;
    Robot_fromScratch robot;

    public enum Articulation {
        MANUAL,
        SCORE_PIXEL,
        FOLD,
    }


    //LIVE STATES
    public Articulation articulation;
    public static double outtakeTicks;


    public Outtake (HardwareMap hardwareMap, Robot_fromScratch robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;

        articulation = Articulation.MANUAL;
    }


    @Override
    public void update(Canvas fieldOverlay) {

    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("outtake position", outtakeTicks);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return null;
    }
}
