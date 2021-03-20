package org.firstinspires.ftc.teamcode.configs;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.HardwareMapV2;

public class teleConfigTESTING_S implements teleOpInterface{
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    double intakeTime, outtakeTime, xTime, yTime, dTime, somePause;
    double perfectval = 0.33;
    int reverseIntake = 1;

    public teleConfigTESTING_S(HardwareMapV2 robot) {
        this.robot = robot;
        drivetrain = new Drivetrain(robot);
    }

    // 0: a=intake in, b=intake out
    // 1: a=intake in, b=outtake out
    // 2: a=bring wobble up/down
    // 3: a=outtake angle up, b=outtake angle down

    int confignumber = 0;
    String configname = "none??";

    public void a(boolean pressed) {
        if (pressed) {
            if (confignumber == 0 || confignumber == 1) {
                if (pressed && System.currentTimeMillis() - intakeTime >= 700) {
                    robot.intake.setPower((robot.intake.getPower() != 0.0) ? 0 : 0.4);
                    intakeTime = System.currentTimeMillis();
                }
            } else if (confignumber == 2) {
                if (pressed && System.currentTimeMillis() - dTime >= 500) {
                    robot.wobble.setPosition((robot.wobble.getPosition() < 0.5 ? 1 : 0));
                    dTime = System.currentTimeMillis();
                }
            } else {
                if (pressed && System.currentTimeMillis() - xTime >= 500) {
                    drivetrain.incrementtilt(-0.01);
                    xTime = System.currentTimeMillis();
                }
            }
        }
    }


    public void b(boolean pressed) {
        if (pressed) {
            if (confignumber == 0) {
                if (pressed && System.currentTimeMillis() - intakeTime >= 700) {
                    robot.intake.setPower((robot.intake.getPower() != 0.0) ? 0 : -0.4);
                    intakeTime = System.currentTimeMillis();
                }
            } else if (confignumber == 1) {
                if (pressed && System.currentTimeMillis() - outtakeTime >= 700) {
                    drivetrain.outtakeAll(0, (((robot.outtake.getPower() >= 0.1) ? 0 : 1)));
                    somePause = System.currentTimeMillis();
                    if (robot.outtake.getPower() >= 0.1) {
                        while (System.currentTimeMillis() - somePause >= 500) {
                        }
                        drivetrain.outtakeAll(1.0);
                    }
                    outtakeTime = System.currentTimeMillis();
                }
            } else if (confignumber == 3) {
                if (pressed && System.currentTimeMillis() - yTime >= 500) {
                    drivetrain.incrementtilt(0.01);
                    yTime = System.currentTimeMillis();
                }
            }
        }
    }

    public void x(boolean pressed) {
        if (pressed) {
            confignumber += 1;
            if (confignumber > 3) {
                confignumber = 0;
            }
            switch (confignumber) {
                case 1:
                    configname = "a = intake in, b = intake out";
                    break;
                case 2:
                     configname = "a = intake in, b = outtake out";
                     break;
                case 3:
                    configname = "a = bring wobble up/down";
                    break;
                case 4:
                    configname = "a = outtake angle up, b = outtake angle down";
            }
        }
    }

    public void y(boolean pressed) {
        if (pressed) {
            robot.setPowerAll(0);
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

    }

    public void rt(float pressure) {

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
        telemetryDM.put("Instructions: ", configname);
    }

    public void loop() {

    }

    public void clearTelemetryDM() {
        telemetryDM.clear();
    }

    public String getName() {
        return "HardwareConfifS";
    }
}
