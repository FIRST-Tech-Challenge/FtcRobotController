package org.firstinspires.ftc.teamcode;

public class teleConfigEx implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    double intakeTime, outtakeTime, xTime, yTime;
    double perfectval = 0.35;

    teleConfigEx (HardwareMapV2 robot) {
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    public void a(boolean pressed) { if (pressed && System.currentTimeMillis()-intakeTime>=700) {robot.intake.setPower((robot.intake.getPower() >= 0.1) ? 0 : 0.6); intakeTime = System.currentTimeMillis();} }

    public void b(boolean pressed) {
        if (pressed && System.currentTimeMillis()-outtakeTime>=700) {drivetrain.outtakeAll(((robot.conveyor.getPower() >= 0.1) ? 0 : 1)); outtakeTime = System.currentTimeMillis();}
    }

    public void x(boolean pressed) {
        if (pressed && System.currentTimeMillis()-xTime>=500) {drivetrain.incrementtilt(-0.01); xTime = System.currentTimeMillis();}
    }

    public void y(boolean pressed) {
        if (pressed && System.currentTimeMillis()-yTime>=500) {drivetrain.incrementtilt(0.01); yTime = System.currentTimeMillis();}
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
        if (pressed){drivetrain.tilt(perfectval);}
    }

    public void rt(float pressure) {

    }

    public void lb(boolean pressed) {
        if (pressed) {
            perfectval = robot.leftTilt.getPosition();
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
        telemetryDM.put("test","test");
        telemetryDM.put("Left Tilt", String.valueOf(robot.leftTilt.getPosition()));
        telemetryDM.put("Right Tilt", String.valueOf(robot.rightTilt.getPosition()));
    }

    public void loop() {

    }

    public String getName() {
        return "teleConfigEx";
    }
}
