package org.firstinspires.ftc.teamcode.configs;

import android.renderscript.RSDriverException;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.HardwareMapV2;

public class NewConfig implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    double intakeTime, outtakeTime, xTime, yTime, dTime, button;
    double intakePwr, outtakePower;
    double limiter, inverse;
    double outtakeAngle = 0.42;
    double collectingAngle = 0.37;
    double savedAngle = 0.44;

    public NewConfig (HardwareMapV2 robot) {
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    public void a(boolean pressed) {
        //Wobble
        if (pressed && System.currentTimeMillis()-button >= 300) {
            robot.wobble.setPosition((robot.wobble.getPosition()<0.05) ? 1.0 : 0.0);
            button = System.currentTimeMillis();
        }
    }

    public void b(boolean pressed) {
        //Intake
        if (pressed && System.currentTimeMillis()-button >= 300) {
            if (robot.leftTilt.getPosition() > collectingAngle){
                drivetrain.tilt(collectingAngle);
            }
            intakePwr = (Math.abs(intakePwr)!=0.9) ? 0.9 : 0.0;
            outtakePower = 0.0;
            button = System.currentTimeMillis();
        }
    }

    public void x(boolean pressed) {
        //Tilt
        if (pressed && System.currentTimeMillis()-button>=300) {
            drivetrain.incrementtilt(-0.01);
            button = System.currentTimeMillis();
        }
    }

    public void y(boolean pressed) {
        //Tilt
        if (pressed && System.currentTimeMillis()-button>=300) {
            drivetrain.incrementtilt(0.01);
            button = System.currentTimeMillis();
        }
    }

    public void dd(boolean pressed) {
        //Outake Rev
        if (pressed && System.currentTimeMillis()-button>=300) {
            outtakePower = (Math.abs(outtakePower)!=1.0) ? 1.0 : 0.0;
            if (robot.leftTilt.getPosition() < outtakeAngle) {
                drivetrain.tilt(outtakeAngle);
            }
            intakePwr = 0.0;
            button = System.currentTimeMillis();
        }
    }

    public void dp(boolean pressed) {
        //Single cycle shoot
        if (pressed && System.currentTimeMillis()-button>=300) {
            drivetrain.invertSingleCycle();
            button = System.currentTimeMillis();
        }
    }

    public void dl(boolean pressed) {

    }

    public void dr(boolean pressed) {

    }

    public void rb(boolean pressed) {
//        if (pressed && System.currentTimeMillis()-button>=300){
//            savedAngle = robot.leftTilt.getPosition();
//            button = System.currentTimeMillis();
//        }
    }

    public void rt(float pressure) {
        if (pressure!=0) {
            limiter = 0.8;
        }else {
            limiter = 1.0;
        }
    }

    public void lb(boolean pressed) {
//        if (pressed && System.currentTimeMillis()-button>=300){
//            drivetrain.tilt(savedAngle);
//            button = System.currentTimeMillis();
//        }
    }

    public void lt(float pressure) {
        if (pressure!=0) {
            inverse = -1.0;
        }else {
            inverse = 1.0;
        }
    }

    public void rjoy(float x, float y) {

    }

    public void ljoy(float x, float y) {

    }

    public void rjoyb(boolean pressed) {

    }

    public void ljoyb(boolean pressed) {

    }

    public void custom1() {

    }

    public void updateTelemetryDM() {

    }

    public void loop() {
        robot.intake.setPower(intakePwr*limiter*inverse);
        robot.outtake.setPower(outtakePower*limiter*inverse);
        outtakeAngle = (robot.getVoltage()>13.5) ? 0.43 : 0.42;
    }

    public void clearTelemetryDM() {
        telemetryDM.clear();
    }

    public String getName() {
        return "OP Config";
    }
}
