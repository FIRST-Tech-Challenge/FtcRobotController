package org.firstinspires.ftc.teamcode.src.Utills;

public abstract class ThreadedSubsystemTemplate extends Thread {
    protected volatile boolean isRunning = true;
    protected long sleepTime = 50;
    protected Executable<Boolean> opModeIsActive;
    protected Executable<Boolean> isStopRequested;

    protected abstract void threadMain();

    public ThreadedSubsystemTemplate(Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested) {
        this.isStopRequested = isStopRequested;
        this.opModeIsActive = opModeIsActive;
    }

    public void end() {
        this.isRunning = false;
    }

    public void run() {
        try {
            while (isRunning && !isStopRequested.call()) {
                this.threadMain();
                Thread.sleep(sleepTime);
            }
        } catch (InterruptedException e) {
            return;
        }

    }

    public boolean isRunning() {
        return this.isRunning;
    }


}
