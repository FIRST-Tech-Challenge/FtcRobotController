package org.firstinspires.ftc.teamcode;

public class teleConfigShrinidhi implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    public void a() {

        if(robot.intake.getPower() == 1.0){
            robot.intake.setPower(0.0);
        }else {
            robot.intake.setPower(1.0);
        }

    }

    public void b() {
        if(robot.outtake.getPower() == 1.0) {
            robot.outtake.setPower(0.0);
        }else {
            robot.outtake.setPower(1.0);
        }
        if(robot.conveyor.getPower() == 1.0){
            robot.conveyor.setPower(0.0);
        }else {
            robot.conveyor.setPower(1.0); 
        }
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

    }
}
