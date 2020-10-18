package org.firstinspires.ftc.teamcode;

public class teleConfigEx implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;

    public void a() { robot.intake.setPower((robot.intake.getPower() >= 0.1) ? 0 : 1); }

    public void b() { drivetrain.outtakeAll((robot.outtake.getPower() >= 0.1) ? 0 : 1); }

    public void x() {

    }

    public void y() {

    }

    public void dd() {

    }

    public void dp() {

    }

    public void dl() {

    }

    public void dr() {

    }

    public void rb() {

    }

    public void rt(float pressure) {

    }

    public void lb() {

    }

    public void lt(float pressure) {

    }

    public void rjoy(float x, float y) {

    }

    public void ljoy(float x, float y) {

    }

    public void rjoyb() {

    }

    public void ljoyb() {

    }

    public void custom1() {

    }

    public void updateTelemetryDM() {
        telemetryDM.put("test","test");
    }

    public String getName() {
        return "teleConfigEx";
    }
}
