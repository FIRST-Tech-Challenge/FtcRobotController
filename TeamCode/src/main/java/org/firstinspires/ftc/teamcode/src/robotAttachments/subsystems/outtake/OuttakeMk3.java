package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.src.utills.Executable;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class OuttakeMk3 {
    private final CRServo outtakeServo;
    private final ColorSensor outtakeSensor;

    private final double rotationPower = 0.25;

    private final ElapsedTime motionTimer = new ElapsedTime();

    private final double sleepPercentage = .9;
    private final ReentrantLock lock = new ReentrantLock();
    private final BlockingQueue<Executable<Void>> taskQueue = new LinkedBlockingQueue<>();
    private long rotationTime = 0;
    private boolean movingForward = false;
    private boolean movingBackward = false;

    public OuttakeMk3(@NonNull final HardwareMap hardwareMap, @NonNull final String ServoName, @NonNull final String SensorName, Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested) {
        this.outtakeSensor = hardwareMap.colorSensor.get(SensorName);
        this.outtakeServo = hardwareMap.crservo.get(ServoName);
        outtakeSensor.enableLed(true);
    }

    /**
     * Rotates the bucket forward to dump
     *
     * @param millis The time the bucket is to rotate for
     */
    public void rotateForward(long millis) {
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

    public void _rotateForward(long millis) throws InterruptedException {
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
            outtakeServo.setPower(0);
            lock.unlock();
            RobotLog.d("Task RF Finished");
        }
    }

    public void rotateBackward(long millis) throws InterruptedException {
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

    public void _rotateBackward(long millis) throws InterruptedException {
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
            outtakeServo.setPower(0);
            RobotLog.d("Task RB Finished");
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

    public void halt() {
        taskQueue.clear();
        this.outtakeServo.setPower(0);
        taskQueue.clear();
    }

    public void update() throws InterruptedException {
        if (!taskQueue.isEmpty()) {
            Executable<Void> task = taskQueue.take();
            RobotLog.d("Task Queue Size: " + taskQueue.size());
            task.call();
        }
    }
}
