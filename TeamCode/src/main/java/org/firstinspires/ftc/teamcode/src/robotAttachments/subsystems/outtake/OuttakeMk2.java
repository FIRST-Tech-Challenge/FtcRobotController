package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.ThreadedSubsystemTemplate;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

/**
 * this is the class for our robot's intake subsystem
 */
public class OuttakeMk2 extends ThreadedSubsystemTemplate implements Outtake {

    protected final ElapsedTime closeTimer = new ElapsedTime();
    private final CRServo outtakeServo;
    private final ColorSensor outtakeSensor;
    private final ElapsedTime motionTimer = new ElapsedTime();
    private final double rotationPower = 0.25;
    private final double sleepPercentage = .9;
    private final long openPos = 1000;
    private final long oneEightyDegreePos = 1500;
    private final BlockingQueue<Executable<Void>> taskQueue = new LinkedBlockingQueue<>();
    private final ReentrantLock lock = new ReentrantLock();
    protected boolean y_depressed2 = true;
    protected boolean b_depressed = true;
    protected boolean a_depressed = true;
    private long rotationTime = 0;
    private boolean movingForward = false;
    private boolean movingBackward = false;


    public OuttakeMk2(@NonNull final HardwareMap hardwareMap, @NonNull final String ServoName, @NonNull final String SensorName, Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested) {
        super(opModeIsActive, isStopRequested);
        this.outtakeSensor = hardwareMap.colorSensor.get(SensorName);
        this.outtakeServo = hardwareMap.crservo.get(ServoName);
        outtakeSensor.enableLed(true);
    }

    /**
     * Rotates the bucket forward to dump
     *
     * @param millis The time the bucket is to rotate for
     */
    private void rotateForward(long millis) {
        taskQueue.add(() -> {
            try {
                this._rotateForward(millis);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return null;
            }
            return null;
        });
    }

    private void _rotateForward(long millis) throws InterruptedException {
        lock.lock();
        try {
            movingForward = true;
            outtakeServo.setPower(rotationPower);
            motionTimer.reset();
            try {
                Thread.sleep((long) (millis * sleepPercentage));
            } catch (InterruptedException e) {
                this.InterruptedExceptionHandler();
                throw e;
            }
            while (motionTimer.milliseconds() < millis) {
                this.safeSpinWait();
            }
            movingForward = false;
        } finally {
            lock.unlock();
        }
    }

    private void rotateBackward(long millis) throws InterruptedException {
        taskQueue.add(() -> {
            try {
                this._rotateBackward(millis);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return null;
            }
            return null;
        });

    }

    private void _rotateBackward(long millis) throws InterruptedException {
        lock.lock();
        try {
            movingBackward = true;
            outtakeServo.setPower(-rotationPower);
            motionTimer.reset();
            try {
                Thread.sleep((long) (millis * sleepPercentage));
            } catch (InterruptedException e) {
                this.InterruptedExceptionHandler();
                throw e;
            }
            while (motionTimer.milliseconds() < millis) {
                this.safeSpinWait();
            }
            movingBackward = false;
        } finally {
            lock.unlock();
        }
    }

    private void InterruptedExceptionHandler() {
        this.halt();
        if (movingForward) {
            rotationTime = rotationTime + (long) motionTimer.milliseconds();
        }
        if (movingBackward) {
            rotationTime = rotationTime - (long) motionTimer.milliseconds();
        }
    }

    private void safeSpinWait() throws InterruptedException {
        Thread.yield();
        if (Thread.currentThread().isInterrupted()) {
            this.InterruptedExceptionHandler();
            throw new InterruptedException();
        }
    }

    public void close() throws InterruptedException {
        goTo(0);
    }

    @Override
    public void goTo(double pos) throws InterruptedException {
        long millis = (long) pos;
        if (millis == rotationTime) {
            return;
        }

        if ((millis - rotationTime) > 0) {
            rotateForward(millis - rotationTime);
        } else {
            rotateBackward(Math.abs(millis - rotationTime));
        }


    }

    public void open() throws InterruptedException {
        goTo(openPos);
    }

    private void ClosedReset() throws InterruptedException {
        goTo(0);
        this.rotateBackward(50);
        this.rotationTime = 0;
    }

    @Nullable
    @Override
    public FreightFrenzyGameObject gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) throws InterruptedException {
        FreightFrenzyGameObject currentObject = FreightFrenzyGameObject.EMPTY;

        if (!gamepad2.y) {
            y_depressed2 = true;
        }
        if (gamepad2.y && y_depressed2) {
            y_depressed2 = false;
            if (this.isClosed()) {
                this.open();
                this.closeTimer.reset();
            } else {
                this.close();
            }
            currentObject = FreightFrenzyGameObject.EMPTY;
        }

        if (!gamepad2.b) {
            b_depressed = true;
        }
        if (gamepad2.b && b_depressed) {
            b_depressed = false;
            if (this.isClosed()) {
                this.goTo(openPos);
                this.closeTimer.reset();
            } else {
                this.close();
            }
            currentObject = FreightFrenzyGameObject.EMPTY;
        }

        if (!gamepad2.a) {
            a_depressed = true;
        }
        if (gamepad2.a && a_depressed) {
            a_depressed = false;
            if (this.isClosed()) {
                this.goTo(oneEightyDegreePos);
                this.closeTimer.reset();
            } else {
                this.close();
            }
            currentObject = FreightFrenzyGameObject.EMPTY;
        }


        if (this.closeTimer.seconds() > 1.25) {
            this.close();
            if (this.isClosed()) {
                currentObject = this.identifyContents();
            }
        }
        return currentObject;

    }

    @Override
    public void halt() {
        taskQueue.clear();
        taskQueue.add(() -> {
            this.outtakeServo.setPower(0);
            return null;
        });

    }

    /**
     * Identifies the contents in the bucket
     *
     * @return The {@link FreightFrenzyGameObject} inside the bucket
     */
    public FreightFrenzyGameObject identifyContents() {
        return FreightFrenzyGameObject.identify(this.getRGB());
    }

    /**
     * A getter for the isClosed boolean
     *
     * @return Returns true if the grabber is closed, false if otherwise
     */
    public boolean isClosed() {
        return this.rotationTime == 0;
    }


    /**
     * this following method takes a parameter for the type of color and outputs the sensor's number for that color
     *
     * @param color the name of the color wanted
     * @return this returns a number of the value for the name of the wanted color
     */
    public int getColor(RGBCameraColors color) {
        switch (color) {
            case Red:
                return outtakeSensor.red();

            case Blue:
                return outtakeSensor.blue();

            case Green:
                return outtakeSensor.green();

            case Alpha:
                return outtakeSensor.alpha();
            default:
                return 0;
        }
    }

    /**
     * Returns what the Color Sensor Sees
     *
     * @return Returns values from 0 to 255 in the form of R,G,B
     */
    public double[] getRGB() {
        return new double[]{outtakeSensor.red(), outtakeSensor.green(), outtakeSensor.blue()};
    }


    public boolean itemInBucket() {
        return !(this.identifyContents() == FreightFrenzyGameObject.EMPTY);
    }

    /**
     * Analyzes the content of the bucket to determine shape, returns the corresponding blink pattern
     *
     * @return Returns the blink pattern for the object in the bucket
     */
    public RevBlinkinLedDriver.BlinkinPattern getLEDPatternFromFreight() {
        return FreightFrenzyGameObject.getLEDColorFromItem(FreightFrenzyGameObject.identify(this.getRGB()));
    }

    @Override
    public void threadMain() throws InterruptedException {
        Executable<Void> task = taskQueue.take();
        task.call();
    }

    @Override
    protected void onEnd() {
        this.halt();
    }
}

