package org.firstinspires.ftc.teamcode;

public class teleConfigRohit2 implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    double limiter2 = 1.0;
    double limiter1 = 1.0;

    public void a(boolean pressed) {

    }

    public void b(boolean pressed) {

    }

    public void x(boolean pressed) {

    }

    public void y(boolean pressed) {
        //outtake synchronous
        if (pressed){
            drivetrain.outtakeAll(1.0*limiter2*limiter1);
            robot.intake.setPower(0);
        }
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
            limiter2 = 0.2;
        }
    }

    public void rt(float pressure) {
        if (pressure!=0){
            limiter1 = 0.5;
        }
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
        return "teleConfigRohit2";
    }
}
