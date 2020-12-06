package org.firstinspires.ftc.teamcode;

public class teleConfigSamih implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    double angle = 0;
    double outlimiter = 1f;

    teleConfigSamih(HardwareMapV2 robot){
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    public void updateTelemetryDM() {

    }

    public void loop() {
        drivetrain.outtakeAll((robot.conveyor.getPower() >= 0.1) ? 0 : 1 * outlimiter, (robot.outtake.getPower() >= 0.1) ? 0 : 1 * outlimiter);

    }

    public String getName() {
        return "teleConfigSamih";
    }

    public void a(boolean pressed) {
        robot.intake.setPower((robot.intake.getPower() >= 0.1) ? 0 : 1);
    }

    public void b(boolean pressed) {
        outlimiter = 1;
    }

    public void x(boolean pressed) {
        robot.intake.setPower(0);
        drivetrain.outtakeAll(0,0);
    }

    public void y(boolean pressed) {
        angle = (angle >= 1) ? 0 : angle + 0.5;
        drivetrain.tilt(angle);

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
        if (pressed) {
            outlimiter = 0.5;
        } else {
            outlimiter = 1;
        }
    }

    public void rt(float pressure) {

    }

    public void lb(boolean pressed) {
        robot.intake.setPower(-0.7);
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
}
