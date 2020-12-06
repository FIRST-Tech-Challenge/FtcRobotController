package org.firstinspires.ftc.teamcode;

public class teleConfigRohit2 implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    enum overrides{INTAKE, CONVEYOR, OUTTAKE, NONE}
    overrides currOverride;
    double limiter2, limiter1, outtakeC, outtakeW, intake = 1.0;
    teleConfigRohit2(HardwareMapV2 robot){
        this.robot = robot;
    }

    public void a(boolean pressed) {
        //wheel out only
        if (pressed) {
            outtakeC = 0.0;
            outtakeW = 0.0;
            intake = 0.0;
            currOverride = overrides.OUTTAKE;
        }
    }

    public void b(boolean pressed) {
        //Intake Synchronous
        if (pressed) {
            outtakeC = 0.0;
            outtakeW = 0.0;
            intake = 1.0;
            currOverride = overrides.NONE;
        }
    }

    public void x(boolean pressed) {
    //intake only
        if (pressed) {
            outtakeC = 0.0;
            outtakeW = 0.0;
            intake = 0.0;
            currOverride = overrides.INTAKE;
        }
    }

    public void y(boolean pressed) {
        //outtake synchronous
        if (pressed){
            outtakeC = 1.0;
            outtakeW = 1.0;
            intake = 0.0;
            currOverride = overrides.NONE;
        }
    }

    public void dd(boolean pressed) {
        if (pressed) {
            drivetrain.tiltpos(Drivetrain.tiltDirect.DOWN);
        }
    }

    public void dp(boolean pressed) {
        if (pressed) {
            drivetrain.tiltpos(Drivetrain.tiltDirect.UP);
        }
    }

    public void dl(boolean pressed) {
        if (pressed) {
            drivetrain.tilt(robot.leftTilt.getPosition()-0.1);
        }
    }

    public void dr(boolean pressed) {
        if (pressed) {
            drivetrain.tilt(robot.leftTilt.getPosition()+0.1);
        }
    }

    public void rb(boolean pressed) {
        limiter1 = (pressed) ? 0.2 : 1.0;
    }

    public void rt(float pressure) {
        limiter2 = (pressure!=0.0) ? 0.5 : 1.0;
    }

    public void lb(boolean pressed) {

    }

    public void lt(float pressure) {

    }

    public void rjoy(float x, float y) {
        switch (currOverride) {
            case NONE:
                break;
            case INTAKE:
                robot.intake.setPower(y);
                break;
            case OUTTAKE:
                robot.outtake.setPower(y);
                break;
            case CONVEYOR:
                robot.conveyor.setPower(y);
                break;
        }
    }

    public void ljoy(float x, float y) {

    }

    public void rjoyb(boolean pressed) {
        //conveyor only
        if (pressed) {
            outtakeC = 0.0;
            outtakeW = 0.0;
            intake = 0.0;
            currOverride = overrides.CONVEYOR;
        }
    }

    public void ljoyb(boolean pressed) {

    }

    public void custom1() {

    }

    public void updateTelemetryDM() {

    }

    public void loop() {
        drivetrain.outtakeAll(outtakeC*limiter1*limiter2, outtakeW*limiter1*limiter2);
        robot.intake.setPower(intake*limiter1*limiter2);
    }

    public String getName() {
        return "teleConfigRohit2";
    }
}
