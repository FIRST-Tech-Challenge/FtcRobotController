package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.ThreadedSubsystemInterface;

public class ContinuousIntakeCB extends ContinuousIntake implements ThreadedSubsystemInterface {

    /**
     * A Lambda object that allows this class to check that the OpMode is active
     */
    protected final Executable<Boolean> opModeIsActive;
    /**
     * A Lambda object that allows this class to check the stop requested condition of the OpMode
     */
    protected final Executable<Boolean> isStopRequested;
    /**
     * A boolean that controlls if the thread is running
     */
    protected volatile boolean isRunning = true;
    /**
     * The time in mills that the thread sleeps for after every call of threadMain
     */
    protected final long sleepTime = 50;
    /**
     * The function to be executed when the bucket detects a object in the bucket
     */
    final Executable<Void> callBack;

    /**
     * A constructor
     *
     * @param hardwareMap     The hardware map to initialize devices from
     * @param motorName       The name of the intake motor
     * @param servoName       The name of the intake servo
     * @param colorSensor     The name of the color sensor
     * @param opModeIsActive  A Lambda object that allows this class to check that the OpMode is active
     * @param isStopRequested A Lambda object that allows this class to check the stop requested condition of the OpMode
     * @param callBack        Lambda object that is called once there is a game object in the bucket
     */
    public ContinuousIntakeCB(HardwareMap hardwareMap, String motorName, String servoName, String colorSensor, Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested, Executable<Void> callBack) {
        super(hardwareMap, motorName, servoName, colorSensor, true);
        this.isStopRequested = isStopRequested;
        this.opModeIsActive = opModeIsActive;
        this.callBack = callBack;
    }

    @Override
    public void threadMain() {
        if (this.identifyContents() != gameObject.EMPTY) {
            callBack.call();
        }
    }

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
        } catch (InterruptedException ignored) {
        }

    }

    /**
     * Returns the running state of the thread
     *
     * @return true if running, false otherwise
     */
    public boolean isRunning() {
        return this.isRunning;
    }
}
