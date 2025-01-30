package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Systems.Input;

public class holdYawSync implements Runnable {
    int pos = 0;
    Input input;
    private boolean running;
    private Thread thread;


    public holdYawSync(Input input) {
        this.thread = new Thread(this);
        this.input = input;
    }

    public void setPos(int pos) {
        this.pos = pos;
    }

    public synchronized void start() {
        if (!running) {
            running = true;
            thread.start();
        }
    }

    public void run() {
        while (running) {
            synchronized (this) {
                setTargetYaw();
            }
        }
    }

    public synchronized void stop() {
        running = false;
    }
    public void setTargetYaw() {
        input.spinToPosition(pos);
    }
}
