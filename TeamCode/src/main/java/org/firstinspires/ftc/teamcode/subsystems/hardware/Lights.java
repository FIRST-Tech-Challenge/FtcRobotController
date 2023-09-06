package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

import java.util.Timer;
import java.util.TimerTask;

public abstract class Lights implements SubSystem {

    DcMotor lights;
    private static final long blink_min=1200; //1 second minimum blink time so no more than once every 1.2 seconds.
    private long last_blink_time=-1;
    Timer blink_timer;

    public Lights(DcMotor lights) {
        this.lights = lights;
    }

    public void blinkOnce(long offtime) {
        if (System.currentTimeMillis() > last_blink_time + blink_min) {
            off();
            new Timer().schedule(new TimerTask() {
                @Override
                public void run() {
                    on();
                }
            },offtime);
            //on();
            last_blink_time = System.currentTimeMillis();
        }

    }

    public void blinkEvery(long millis) {
        blink_timer = new Timer();
        blink_timer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                blinkOnce(millis);
            }
        },0,millis);
    }

    public void blinkStop() {
        blink_timer.cancel();
    }

    public void blueOn() {
        lights.setPower(-1);
    }

    public void off() {
        lights.setPower(0);
    }

    public abstract void on();

    public void redOn() {
        lights.setPower(1);
    }

}
