package org.firstinspires.ftc.teamcode;

public class teleConfigEx implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    double intakeTime;

    teleConfigEx (HardwareMapV2 robot) {
        this.robot = robot;
    }

    public void a(boolean pressed) { if (pressed && System.currentTimeMillis()-intakeTime>=1000) {robot.intake.setPower((robot.intake.getPower() >= 0.1) ? 0 : 0.6); intakeTime = System.currentTimeMillis();} }

    public void b(boolean pressed) { if (pressed) {drivetrain.outtakeAll((robot.outtake.getPower() >= 0.1) ? 0 : 1);} }

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
        telemetryDM.put("test","test");
    }

    public void loop() {

    }

    public String getName() {
        return "teleConfigEx";
    }
}
