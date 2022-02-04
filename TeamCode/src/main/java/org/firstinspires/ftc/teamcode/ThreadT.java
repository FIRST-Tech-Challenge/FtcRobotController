package org.firstinspires.ftc.teamcode;

public class ThreadT extends Thread {
    MecanumChassis robot;

    ThreadT(MecanumChassis x){
        robot = x;

        start();
    }

    int waitFirst;
    int waitSecond;
    double power;

    private volatile boolean running = true;
    private volatile boolean paused = true;
    private final Object pauseLock = new Object();

    public void run() {
        while (running) {
            synchronized (pauseLock) {
                if (!running) break;
                if (paused) {
                    try {
                        synchronized (pauseLock) {
                            pauseLock.wait();
                        }
                    } catch (InterruptedException ex) {
                        break;
                    }
                    if (!running) break;
                }
            }
            // Code here +

            try {
                sleep(waitFirst);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            robot.intakeUp.setPower(power);

            try {
                sleep(waitSecond);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            pause();
            // Code here -
        }

    }
    public void killT() {
        running = false;
        round();
    }
    public void pause() {
        paused = true;
    }
    public void round() {
        synchronized (pauseLock) {
            paused = false;
            pauseLock.notifyAll();
        }
    }
}
