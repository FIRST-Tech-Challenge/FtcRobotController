package org.firstinspires.ftc.teamcode;

public class teleConfigTest implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    static boolean wait = false;
    static boolean up = false;

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

    public void rs() {

    }

    public void lb() {

    }

    public void ls() {

    }

    public void rjoy(float x, float y) {
        if (Math.abs(y)>0.75){ wait = true; up = y > 0; }
        if (y == 0 && wait){ drivetrain.tiltpos((up) ? Drivetrain.tiltDirect.UP : Drivetrain.tiltDirect.DOWN); wait = false;}
    }

    public void ljoy(float x, float y) {

    }

    public void custom1() {

    }
}
