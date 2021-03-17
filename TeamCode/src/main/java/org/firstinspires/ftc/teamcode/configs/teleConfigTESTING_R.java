package org.firstinspires.ftc.teamcode.configs;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.HardwareMapV2;

public class teleConfigTESTING_R implements teleOpInterface{

    HardwareMapV2 robot;
    Drivetrain drivetrain;
    enum overrides{INTAKE, CONVEYOR, OUTTAKE, NONE}
    teleConfigRohit2.overrides currOverride;
    double limiter2, limiter1, outtakeC, outtakeW, intake = 1.0;
    double intakeTime, outtakeTime, xTime, yTime, dTime, button, dpad;

    public teleConfigTESTING_R(HardwareMapV2 robot){
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    public void a(boolean pressed) {
        if (pressed && System.currentTimeMillis()-button >= 500){
            robot.intake.setPower((robot.intake.getPower() == 0) ? 1.0 : 0.0);
            button = System.currentTimeMillis();
        }
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
        drivetrain.outtakeAll(outtakeC*limiter1*limiter2, outtakeW*limiter1*limiter2);
        robot.intake.setPower(intake*limiter1*limiter2);
    }

    public String getName() {
        return null;
    }
}
