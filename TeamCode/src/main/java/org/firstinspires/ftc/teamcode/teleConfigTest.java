package org.firstinspires.ftc.teamcode;

public class teleConfigTest implements teleOpInterface {
    HardwareMapV2 robot;

    public void a() {
        robot.intake.setPower((robot.intake.getPower() >= 0.1) ? 0 : 1);
    }

    public void b() {
        robot.outtake.setPower((robot.outtake.getPower() >= 0.1) ? 0 : 1);
    }

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

    }

    public void ljoy(float x, float y) {

    }
}
