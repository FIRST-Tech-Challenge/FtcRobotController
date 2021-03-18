package org.firstinspires.ftc.teamcode.configs;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.HardwareMapV2;

public class teleConfigTESTING_S implements teleOpInterface{
    HardwareMapV2 robot;
    Drivetrain drivetrain;

    public teleConfigTESTING_S(HardwareMapV2 robot) {
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    public void a(boolean pressed) {

    }

    public void b(boolean pressed) {

    }

    public void x(boolean pressed) {

    }

    public void y(boolean pressed) {

    }

    public void dd(boolean pressed) {

    }

    public void dp(boolean pressed) {

    }

    public void dl(boolean pressed) {

    }

    public void dr(boolean pressed) {

    }

    public void rb(boolean pressed) {

    }

    public void rt(float pressure) {

    }

    public void lb(boolean pressed) {

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

    }

    public void loop() {

    }

    public String getName() {
        return null;
    }
}
