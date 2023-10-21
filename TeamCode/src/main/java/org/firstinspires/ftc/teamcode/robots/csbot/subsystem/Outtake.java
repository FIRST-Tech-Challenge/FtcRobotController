package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

public class Outtake implements Subsystem {

    HardwareMap hardwareMap;
    Robot robot;

    private DcMotorEx slide = null;
    private Servo pixelFliper = null;
    int slidePosition;
    public static int slidePositionMax = 2000;
    int slideSpeed = 20;
    public static int UNFLIPPEDPOSITION = 2000;
    public static int FLIPPEDPOSITION = 950;
    private boolean flipped = false;

    public enum Articulation {
        MANUAL,
        SCORE_PIXEL,
        FOLD,
    }


    //LIVE STATES
    public Articulation articulation;
    public static double outtakeTicks;


    public Outtake (HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;

        slide = this.hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMotorEnable();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        pixelFliper = this.hardwareMap.get(Servo.class, "pixelFlipper");

        articulation = Articulation.MANUAL;
    }

    public void moveSlide(int power)
    {
        slidePosition += power*slideSpeed;
        if(slidePosition < 0)
            slidePosition = 0;
        if (slidePosition> slidePositionMax) slidePosition=slidePositionMax;
        slide.setTargetPosition(slidePosition);
    }

    public void flip()
    {
        if(flipped) {
            pixelFliper.setPosition(Utils.servoNormalize(UNFLIPPEDPOSITION));
            flipped = false;
        }
        else {
            pixelFliper.setPosition(Utils.servoNormalize(FLIPPEDPOSITION));
            flipped = true;
        }
    }


    @Override
    public void update(Canvas fieldOverlay) {

    }

    @Override
    public void stop() {
        slide.setMotorDisable();
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
