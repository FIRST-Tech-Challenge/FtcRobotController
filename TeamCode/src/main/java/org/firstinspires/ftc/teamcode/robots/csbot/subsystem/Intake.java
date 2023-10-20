package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Joint;

import java.util.LinkedHashMap;
import java.util.Map;

//i still have no clue what the intake needs lmao

public class Intake implements Subsystem {
    //CONSTANTS
    HardwareMap hardwareMap;
    Robot_fromScratch robot;
    Servo diverter;
    Joint beaterBarAngleController;
    DcMotorEx beaterBar;
    public static double BEATER_BAR_ADJUST_SPEED = 5;
    public static double BEATER_BAR_INTAKE_VELOCITY = 1.0;
    public static double BEATER_BAR_EJECT_VELOCITY = -1.0;
    public static double BEATER_BAR_ANGLE_CONTROLLER_HOME;
    public static double BEATER_BAR_ANGLE_CONTROLLER_TICKS_PER_DEGREE;
    public static double BEATER_BAR_ANGLE_CONTROLLER_MIN_DEGREES;
    public static double BEATER_BAR_ANGLE_CONTROLLER_MAX_DEGREES;
    public static double BEATER_BAR_ANGLE_CONTROLLER_START_ANGLE;
    public static double BEATER_BAR_ANGLE_CONTROLLER_SPEED;
    public static double BEATER_BAR_FOLD_ANGLE;
    public static double BEATER_BAR_GROUND_ANGLE;


    public enum Articulation {
        GROUND_INTAKE,
        STACK_INTAKE,
        EJECT,
        OFF,
        FOLD,
        MANUAL,
    }


    //LIVE STATES
    public Articulation articulation;
    public static int numPixelsInStack = 6;
    public static double angleToStack;
    public static double beaterBarTargetAngle;


    public Intake(HardwareMap hardwareMap, Robot_fromScratch robot) {
            this.hardwareMap = hardwareMap;
            this.robot = robot;
            articulation = Articulation.FOLD;

            diverter = hardwareMap.get(Servo.class, "diverter");
            beaterBar = hardwareMap.get(DcMotorEx.class, "beaterBar");

            beaterBarAngleController = new Joint(hardwareMap, "beaterBarAngleController", false, BEATER_BAR_ANGLE_CONTROLLER_HOME, BEATER_BAR_ANGLE_CONTROLLER_TICKS_PER_DEGREE, BEATER_BAR_ANGLE_CONTROLLER_MIN_DEGREES, BEATER_BAR_ANGLE_CONTROLLER_MAX_DEGREES, BEATER_BAR_ANGLE_CONTROLLER_START_ANGLE, BEATER_BAR_ANGLE_CONTROLLER_SPEED);
            beaterBar.setMotorEnable();
            beaterBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void update(Canvas fieldOverlay) {
        switch (articulation) {
            case MANUAL:
                break;
            case OFF:
                beaterBar.setPower(0);
                break;
            case GROUND_INTAKE:
                beaterBar.setVelocity(BEATER_BAR_INTAKE_VELOCITY);
                beaterBarTargetAngle = BEATER_BAR_GROUND_ANGLE;
                break;
            case EJECT:
                beaterBar.setVelocity(BEATER_BAR_EJECT_VELOCITY);
                break;
            case FOLD:
                beaterBar.setPower(0);
                beaterBarTargetAngle = BEATER_BAR_FOLD_ANGLE;
                break;
            case STACK_INTAKE:
                beaterBar.setPower(BEATER_BAR_INTAKE_VELOCITY);
                beaterBarTargetAngle = angleToStack;
                break;
        }
        beaterBarAngleController.setTargetAngle(beaterBarTargetAngle);
        beaterBarAngleController.update();
    }

    public Articulation articulate(Articulation target) {
        articulation = target;
        return articulation;
    }

    public double adjustBeaterBarAngle(double speed) {
        beaterBarTargetAngle += speed * BEATER_BAR_ADJUST_SPEED;
        return beaterBarTargetAngle;
    }

    public void toggleBeaterBar() {
        if(Math.abs(beaterBar.getVelocity() - BEATER_BAR_INTAKE_VELOCITY) < .2)
        beaterBar.setVelocity(0);
        else
            beaterBar.setVelocity(BEATER_BAR_INTAKE_VELOCITY);
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
