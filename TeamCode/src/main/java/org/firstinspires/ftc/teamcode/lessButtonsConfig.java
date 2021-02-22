package org.firstinspires.ftc.teamcode;

public class lessButtonsConfig implements teleOpInterface {

    HardwareMapV2 robot;
    Drivetrain drivetrain;
    double limiter2, limiter1, outtakeC, outtakeW, intake = 1.0;
    double intakeTime, outtakeTime, xTime, yTime, dTime, button, dpad;
    enum overrides{INTAKE, CONVEYOR, OUTTAKE, NONE}
    teleConfigRohit2.overrides currOverride;

    lessButtonsConfig(HardwareMapV2 robot){
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    public void a(boolean pressed) {
        if (pressed && System.currentTimeMillis()-button>=500) {
            drivetrain.incrementtilt(-0.01);
            button = System.currentTimeMillis();
        }
    }

    public void b(boolean pressed) {
        if (pressed && System.currentTimeMillis()-button>=500) {
            outtakeC = (intake == 0.0) ? 1.0 : 0.0;
            outtakeW = outtakeC;
            intake = (outtakeC == 0.0) ? 1.0 : 0.0;
            currOverride = teleConfigRohit2.overrides.NONE;
            button = System.currentTimeMillis();
        }
    }

    public void x(boolean pressed) {
        if (pressed && System.currentTimeMillis()-button>=500) {
            robot.wobble.setPosition((robot.wobble.getPosition() < 0.5) ? 1.0 : 0.0);
            button = System.currentTimeMillis();
        }
    }

    public void y(boolean pressed) {
        if (pressed && System.currentTimeMillis()-button>=500) {
            drivetrain.incrementtilt(0.01);
            button = System.currentTimeMillis();
        }
    }

    public void dd(boolean pressed) {

    }

    public void dp(boolean pressed) {

    }

    public void dl(boolean pressed) {

    }

    public void dr(boolean pressed) {
        if (pressed){drivetrain.tilt(0.35);}
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
        return "Less Buttons Config";
    }
}
