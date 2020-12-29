package org.firstinspires.ftc.teamcode;

public class teleConfigRohit2 implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    enum overrides{INTAKE, CONVEYOR, OUTTAKE, NONE}
    overrides currOverride;
    double limiter2, limiter1, outtakeC, outtakeW, intake = 1.0;
    double intakeTime, outtakeTime, xTime, yTime, dTime, button, dpad;
    teleConfigRohit2(HardwareMapV2 robot){
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    public void a(boolean pressed) {
        //wheel out only
        if (pressed && System.currentTimeMillis()-button>=500) {
            outtakeC = 0.0;
            outtakeW = 0.0;
            intake = 0.0;
            currOverride = overrides.OUTTAKE;
            button = System.currentTimeMillis();
        }
    }

    public void b(boolean pressed) {
        //Intake Synchronous
        if (pressed && System.currentTimeMillis()-button>=500) {
            outtakeC = 0.0;
            outtakeW = 0.0;
            intake = 1.0;
            currOverride = overrides.NONE;
            button = System.currentTimeMillis();
        }
    }

    public void x(boolean pressed) {
    //intake only
        if (pressed && System.currentTimeMillis()-button>=500) {
            outtakeC = 0.0;
            outtakeW = 0.0;
            intake = 0.0;
            currOverride = overrides.INTAKE;
            button = System.currentTimeMillis();
        }
    }

    public void y(boolean pressed) {
        //outtake synchronous
        if (pressed && System.currentTimeMillis()-button>=500){
            outtakeC = 1.0;
            outtakeW = 1.0;
            intake = 0.0;
            currOverride = overrides.NONE;
            button = System.currentTimeMillis();
        }
    }

    public void dd(boolean pressed) {
        if (pressed && System.currentTimeMillis()-dpad>=500) {
            drivetrain.tiltpos(Drivetrain.tiltDirect.DOWN);
        }
    }

    public void dp(boolean pressed) {
        if (pressed && System.currentTimeMillis()-dpad>=500) {
            drivetrain.tiltpos(Drivetrain.tiltDirect.UP);
            dpad = System.currentTimeMillis();
        }
    }

    public void dl(boolean pressed) {
        if (pressed && System.currentTimeMillis()-dpad>=500) {
            drivetrain.incrementtilt(-0.01);
            dpad = System.currentTimeMillis();

        }
    }

    public void dr(boolean pressed) {
        if (pressed && System.currentTimeMillis()-dpad>=500) {
            drivetrain.incrementtilt(0.01);
            dpad = System.currentTimeMillis();
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
