package org.firstinspires.ftc.teamcode.lib.control;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.Robot;

import java.util.function.Supplier;

public class MecanumDriveRunnableLQR implements Runnable {
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private Supplier<MecanumDriveMPC> lqr;
    private boolean canUpdate;
    private boolean stop;

    public MecanumDriveMPC lqrDrivetrain;
    public double policyTime;
    long start;

    public MecanumDriveRunnableLQR(Supplier<MecanumDriveMPC> lqr) {
        timeProfiler = new TimeProfiler(false);
        policyTimeProfiler = new TimeProfiler(false);
        this.lqr = lqr;
        canUpdate = false;
        stop = false;
        policyTime = 0d;
    }

    @Override
    public void run() {
        timeProfiler.start();
        while(!stop) {
            if(!canUpdate) {
                policyTimeProfiler.start();
                lqrDrivetrain = lqr.get();
                timeProfiler.update(true);
                try {
                    Thread.sleep(10);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }

                canUpdate = true;
            }

            try {
                Thread.sleep(1);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void updateMPC() {
        if(canUpdate && lqrDrivetrain != null) {
            Robot.setDriveMPC(lqrDrivetrain);
            policyTime = policyTimeProfiler.getDeltaTime(TimeUnits.SECONDS, true);
            canUpdate = false;
        }
    }

    public double getTimeStamp() {
        return timeProfiler.getDeltaTime(TimeUnits.SECONDS);
    }

    public void stop() {
        stop = true;
    }
}
