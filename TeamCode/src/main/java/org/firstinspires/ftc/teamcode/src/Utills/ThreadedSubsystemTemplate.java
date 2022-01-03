package org.firstinspires.ftc.teamcode.src.Utills;

/**
 * This is a template for all subsystems that need threading, provides thread safety
 */
public abstract class ThreadedSubsystemTemplate extends Thread {

    /**
     * A boolean that controlls if the thread is running
     */
    protected volatile boolean isRunning = true;

    /**
     * The time in mills that the thread sleeps for after every call of threadMain
     */
    protected long sleepTime = 50;

    /**
     * A Lambda object that allows this class to check that the OpMode is active
     */
    protected Executable<Boolean> opModeIsActive;

    /**
     * A Lambda object that allows this class to check the stop requested condition of the OpMode
     */
    protected Executable<Boolean> isStopRequested;

    /**
     * A constructor that instantiates the Executable Objects, they allow the thread to end with the OpMode
     *
     * @param isStopRequested A Executable object wrapped around OpMode.isStopRequested()
     * @param opModeIsActive  A Executable object wrapped around OpMode.opModeIsActive()
     */
    public ThreadedSubsystemTemplate(Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested) {
        this.isStopRequested = isStopRequested;
        this.opModeIsActive = opModeIsActive;
    }

    /**
     * It is the main function of the thread
     */
    protected abstract void threadMain();

    /**
     * Ends the life of this thread
     */
    public void end() {
        this.isRunning = false;
    }

    /**
     * This is the method where the thread starts, do not override
     */
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

    /**
     * Returns the running state of the thread
     */
    public boolean isRunning() {
        return this.isRunning;
    }


}
