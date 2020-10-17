package org.firstinspires.ftc.teamcode;

public class teleConfigSamih implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    double angle = 0;
    double outlimiter = 1f;

    public void loop() {
        outlimiter = 1;
    }

    public void updateTelemetryDM() {

    }

    public void a() {
        robot.intake.setPower((robot.intake.getPower() >= 0.1) ? 0 : 1);
    }

    public void b() {
        drivetrain.outtakeAll((robot.conveyor.getPower() >= 0.1) ? 0 : 1 * outlimiter, (robot.outtake.getPower() >= 0.1) ? 0 : 1 * outlimiter);
    }

    public void x() {
        robot.intake.setPower(0);
        drivetrain.outtakeAll(0,0);
    }

    public void y() {
        angle = (angle >= 1) ? 0 : angle + 0.5;
        drivetrain.tilt(angle);

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
        outlimiter = 0.5;
    }

    public void rt() {

    }

    public void lb() {
        robot.intake.setPower(-0.7);
    }

    public void lt() {

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
}
