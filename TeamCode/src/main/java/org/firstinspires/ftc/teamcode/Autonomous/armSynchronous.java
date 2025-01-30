package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Systems.Input;

public class armSynchronous implements Runnable {
    int pos = 0;
    Input input;
    private boolean running;
    private Thread thread;


    public armSynchronous(Input input) {
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
                moveArm();
            }
        }
    }

    public synchronized void stop() {
        running = false;
    }
    public void moveArm() {
        input.setArmPosition(pos);
    }

    public void extendArm(int upArmPos, int armPos) {
        while (armPos != 0 && armPos != pos) {
        }

        while (input.getUpArmPos() > upArmPos) {
            input.upArm(-50);
        }

        input.upArm(0);
    }

    public void retractArm(int upArmPos, int armPos) {
        while (armPos != 0 && armPos != pos) {
        }

        while (input.getUpArmPos() < upArmPos) {
            input.upArm(50);
        }

        input.upArm(0);
    }
}
