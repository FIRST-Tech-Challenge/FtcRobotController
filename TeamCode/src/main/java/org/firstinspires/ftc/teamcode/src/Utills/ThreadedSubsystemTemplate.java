package org.firstinspires.ftc.teamcode.src.Utills;

public abstract class ThreadedSubsystemTemplate implements Runnable {
    protected volatile boolean isRunning = true;
    protected long sleepTime = 50;

    protected abstract void threadMain();

    public void stop() {
        this.isRunning = false;
    }

    public void run() {
        while (isRunning) {
            this.threadMain();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                return;
            }
        }
    }


}
