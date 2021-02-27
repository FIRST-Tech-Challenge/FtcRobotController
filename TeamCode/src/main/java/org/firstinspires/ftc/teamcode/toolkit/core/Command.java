package org.firstinspires.ftc.teamcode.toolkit.core;

import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

public abstract class Command implements Runnable{

    Thread t;
    UpliftTele opMode;
    Subsystem subsystem;
    public boolean isRunning = false;

    public Command(UpliftTele opMode, Subsystem subsystem) {
        this.t = new Thread(this);
        this.opMode = opMode;
        this.subsystem = subsystem;

    }
    public void enable() {
        t.start();
    }

    public void forceStop() {
        opMode.forceStop = true;
        t.interrupt();
        finish();
    }

    public abstract void init();

    public abstract void start();

    public abstract void loop();

    public abstract void stop();

    @Override
    public void run() {
        isRunning = true;

        init();

        subsystem.enable();

        opMode.waitForStart();

        start();

        while(!isLooping() && !opMode.forceStop){
            try {
                Utils.sleep(1);
            } catch (Error e) {
                e.printStackTrace();
            }
        }

        while(!isFinished() && !opMode.forceStop && opMode.opModeIsActive()){
            loop();
        }

        subsystem.disable();

        stop();
        subsystem.stop();
        finish();
    }

    private boolean isStarted(){
        return opMode.isStarted;

    }

    public boolean isLooping() {
        return opMode.isLooping;
    }

    public boolean isFinished() {
        return opMode.isFinished;
    }

    private void finish() {
        isRunning = false;
        t = null;
    }
}
