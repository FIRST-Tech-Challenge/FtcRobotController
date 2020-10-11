package org.firstinspires.ftc.teamcode;

public class teleConfigRohit implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    static boolean wait = false;
    static boolean up = false;
    enum mode{
        INCREMENT, POSITION
    }
    static mode tiltmode = mode.POSITION;

    public void a() {

    }

    public void b() {

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

    public void rt() {

    }

    public void lb() {

    }

    public void lt() {

    }

    public void rjoy(float x, float y) {
        if (Math.abs(y)>0.75){ wait = true; up = y > 0; }
        if (y==0 && wait) {
            switch (tiltmode) {
                case POSITION:
                        drivetrain.tiltpos((up) ? Drivetrain.tiltDirect.UP : Drivetrain.tiltDirect.DOWN);
                    break;
                case INCREMENT:
                    drivetrain.tilt(robot.leftTilt.getPosition() + ((up) ? 0.1 : -0.1));
                    break;
            }
            wait = false;
        }
    }

    public void ljoy(float x, float y) {

    }

    public void rjoyb() {
        tiltmode = (tiltmode==mode.POSITION) ? mode.INCREMENT : mode.POSITION;
    }

    public void ljoyb() {

    }

    public void custom1() {

    }
}
