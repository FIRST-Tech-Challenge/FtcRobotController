package org.firstinspires.ftc.teamcode.src.Utills;

public abstract class ThreadedSubsystemTemplate implements Runnable {
    protected volatile boolean isRunning = true;
    protected long sleepTime = 50;
    protected Executable<Boolean> opModeIsActive;
    protected Executable<Boolean> isStopRequested;

    protected abstract void threadMain();

    public void stop() {
        this.isRunning = false;
    }

    public void run() {
        try {
            while (isRunning && !isStopRequested.call()) {
                this.threadMain();
                Thread.sleep(sleepTime);
            }
        } catch (NullPointerException | InterruptedException e) {
            e.printStackTrace();
        }

    }

    public boolean isRunning() {
        return this.isRunning;
    }


}
