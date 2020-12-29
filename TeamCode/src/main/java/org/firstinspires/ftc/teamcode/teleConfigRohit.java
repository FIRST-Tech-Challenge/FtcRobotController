package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class teleConfigRohit implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    static boolean wait = false;
    static boolean up = false;
    enum mode{
        INCREMENT, POSITION
    }
    static mode tiltmode = mode.POSITION;
    static double inttakePower = 0.0;
    static double outtakePower = 0.0;
    static double conveyorPower = 1.0;

    teleConfigRohit(HardwareMapV2 robot){
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    public void a(boolean pressed) {
        if (pressed) {conveyorPower-=0.1;}
    }

    public void b(boolean pressed) {

    }

    public void x(boolean pressed) {
        if (pressed) {conveyorPower+=0.1;}
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
        if (pressed) {drivetrain.outtakeAll(conveyorPower, outtakePower);}
    }

    public void rt(float pressure) {
        if (pressure>0) { drivetrain.outtakeAll(-conveyorPower, -outtakePower); }
    }

    public void lb(boolean pressed) {
        if (pressed) {robot.intake.setPower(inttakePower);}
    }

    public void lt(float pressure) {
        if (pressure>0) { robot.intake.setPower(-inttakePower); }
    }

    public void rjoy(float x, float y) {
        if (Math.abs(y)>0.75){ wait = true; up = y > 0; }
        if (y==0 && wait) {
            switch (tiltmode) {
                case POSITION:
                    drivetrain.tiltpos((up) ? Drivetrain.tiltDirect.UP : Drivetrain.tiltDirect.DOWN);
                    break;
                case INCREMENT:
                    drivetrain.incrementtilt((up) ? 0.01 : -0.01);
                    break;
            }
            wait = false;
        }
    }

    public void ljoy(float x, float y) {
        if (Math.abs(y)>0.75){ wait = true; up = y > 0; }
        if (y==0 && wait) {
            inttakePower = (up && inttakePower!=1.0) ? inttakePower*2 : (!up && inttakePower!=0.125) ? inttakePower/2 : 1.0;
        }
        wait = false;
    }

    public void rjoyb(boolean pressed) {
        if (pressed) {tiltmode = (tiltmode==mode.POSITION) ? mode.INCREMENT : mode.POSITION;}
    }

    public void ljoyb(boolean pressed) {
        if (pressed) {inttakePower = 1.0;}
    }

    public void custom1() {

    }

    public void updateTelemetryDM() {
        telemetryDM.put("Intake Power: ", String.valueOf(inttakePower));
        telemetryDM.put("Outtake Power: ", String.valueOf(outtakePower));
        telemetryDM.put("Conveyor Power: ", String.valueOf(conveyorPower));
        telemetryDM.put("Tilt Angle: ", String.valueOf(robot.leftTilt.getPosition()));
        telemetryDM.put("Tilt mode: ", String.valueOf(tiltmode));
    }

    @Override
    public void loop() {}

    public String getName() {
        return "teleConfigRohit";
    }


}
