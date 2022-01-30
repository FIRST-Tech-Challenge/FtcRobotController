package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

public class ThreadT extends Thread {
    MecanumChassis robot;

    int waitTime;
    double power;

    private volatile boolean running = true;
    private volatile boolean paused = false;
    private final Object pauseLock = new Object();

    public void run() {
        while (running) {
            synchronized (pauseLock) {
                if (!running) {break;}
                if (paused) {
                    try {
                        synchronized (pauseLock) {
                            pauseLock.wait();
                        }
                    } catch (InterruptedException ex) {
                        break;
                    }
                    if (!running) {break;}
                }
            }
            // Code here +

            try {
                sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            this.robot.intakeUp.setPower(power);

            pause();
            // Code here -
        }

    }
    public void killT() {
        running = false;
        unpause();
    }
    public void pause() {
        paused = true;
    }
    public void unpause() {
        synchronized (pauseLock) {
            paused = false;
            pauseLock.notifyAll();
        }
    }
}
