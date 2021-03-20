package org.firstinspires.ftc.teamcode.configs;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.HardwareMapV2;

public class teleConfigTESTING_R implements teleOpInterface{

    HardwareMapV2 robot;
    Drivetrain drivetrain;
    enum overrides{INTAKE, CONVEYOR, OUTTAKE, NONE}
    teleConfigRohit2.overrides currOverride;
    double varPower, switcher, outtakeC, outtakeW, intake = 1.0;
    double intakeTime, outtakeTime, xTime, yTime, dTime, button, dpad, trigbut;

    public teleConfigTESTING_R(HardwareMapV2 robot){
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    public void a(boolean pressed) {
        //Intake
        if (pressed && System.currentTimeMillis()-button >= 500){
            intake = (robot.intake.getPower() == 0) ? 1.0 : 0.0;
            button = System.currentTimeMillis();
        }
    }

    public void b(boolean pressed) {
        //Outtake
        if (pressed && System.currentTimeMillis()-button >= 500){
            outtakeW = (robot.outtake.getPower() == 0) ? 1.0 : 0.0;
            button = System.currentTimeMillis();
        }
    }

    public void x(boolean pressed) {
        //Conveyor
        if (pressed && System.currentTimeMillis()-button >= 500){
            robot.slapper.setPosition((robot.slapper.getPosition() == 0) ? 1.0 : 0.0);
            button = System.currentTimeMillis();
        }
    }

    public void y(boolean pressed) {
        //Invert set powers from pos to negative
        if (pressed && System.currentTimeMillis()-button >= 500){
            switcher = ((switcher == 1) ? -1 : 1);
        }
    }

    public void dd(boolean pressed) {
        if (pressed && System.currentTimeMillis()-dpad>=500) {
            drivetrain.tiltpos(Drivetrain.tiltDirect.DOWN);
        }
    }

    public void dp(boolean pressed) {
        if (pressed && System.currentTimeMillis()-dpad>=500) {
            drivetrain.tiltpos(Drivetrain.tiltDirect.UP);
            dpad = System.currentTimeMillis();
        }
    }

    public void dl(boolean pressed) {
        if (pressed && System.currentTimeMillis()-dpad>=500) {
            drivetrain.incrementtilt(-0.01);
            dpad = System.currentTimeMillis();

        }
    }

    public void dr(boolean pressed) {
        if (pressed && System.currentTimeMillis()-dpad>=500) {
            drivetrain.incrementtilt(0.01);
            dpad = System.currentTimeMillis();
        }
    }

    public void rb(boolean pressed) {
        if (pressed && System.currentTimeMillis()-trigbut >= 500){
            if (varPower!=1.0) {
                varPower+=0.1;
            }
            button = System.currentTimeMillis();
        }
    }

    public void rt(float pressure) {

    }

    public void lb(boolean pressed) {
        if (pressed && System.currentTimeMillis()-trigbut >= 500){
            if (varPower!=0.0) {
                varPower-=0.1;
            }
            button = System.currentTimeMillis();
        }
    }

    public void lt(float pressure) {

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
        telemetryDM.put("Hardware: ","DONT BREAK IT");
        telemetryDM.put("Left Tilt ", String.valueOf(robot.leftTilt.getPosition()));
        telemetryDM.put("Right Tilt ", String.valueOf(robot.rightTilt.getPosition()));
        telemetryDM.put("Wobble ", String.valueOf(robot.wobble.getPosition()));
        telemetryDM.put("Intake ", String.valueOf(robot.intake.getPower()));
        telemetryDM.put("Outtake ", String.valueOf(robot.outtake.getPower()));
        telemetryDM.put("Set Power ", String.valueOf(varPower));
        telemetryDM.put("Invert ", String.valueOf(switcher));
    }

    public void loop() {
        drivetrain.outtakeAll(outtakeC* switcher * varPower, outtakeW* switcher * varPower);
        robot.intake.setPower(intake* switcher * varPower);
    }

    public String getName() {
        return "HardwareTesting_R";
    }
}
