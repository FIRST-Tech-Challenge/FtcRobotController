package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedHashMap;
import java.util.Map;

//i still have no clue what the intake needs lmao

public class Intake implements Subsystem {
    //CONSTANTS
    HardwareMap hardwareMap;
    Robot_fromScratch robot;
    Servo diverter, beaterBarAngleController;
    DcMotorEx beaterBar;
    public static double BEATER_BAR_INTAKE_VELOCITY = 1.0;
    public static double BEATER_BAR_EJECT_VELOCITY = -1.0;


    public enum Articulation {
        INTAKE,
        EJECT,
        OFF,
        FOLD,
    }


    //LIVE STATES
    public Articulation articulation;


    public Intake(HardwareMap hardwareMap, Robot_fromScratch robot) {
            this.hardwareMap = hardwareMap;
            this.robot = robot;
            articulation = Articulation.OFF;

            diverter = hardwareMap.get(Servo.class, "diverter");
            beaterBarAngleController = hardwareMap.get(Servo.class, "beaterBarAngleController");
            beaterBar = hardwareMap.get(DcMotorEx.class, "beaterBar");

            beaterBar.setMotorEnable();
            beaterBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void update(Canvas fieldOverlay) {
        switch (articulation) {
            case OFF:
                beaterBar.setPower(0);
                break;
            case INTAKE:
                beaterBar.setVelocity(BEATER_BAR_INTAKE_VELOCITY);
                break;
            case EJECT:
                beaterBar.setVelocity(BEATER_BAR_EJECT_VELOCITY);
                break;
        }
    }

    public Articulation articulate(Articulation target) {
        articulation = target;
        return articulation;
    }

    @Override
    public void stop() {
        beaterBar.setMotorDisable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("beater bar amps", beaterBar.getPower());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "INTAKE";
    }
}
