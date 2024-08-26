package com.acmerobotics.dashboard.testopmode;

import com.acmerobotics.dashboard.RobotStatus;
import com.acmerobotics.dashboard.SendFun;

public abstract class TestOpMode {
    private final String name;
    private RobotStatus.OpModeStatus status = RobotStatus.OpModeStatus.STOPPED;

    SendFun sendFun;

    protected TestOpMode(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public RobotStatus.OpModeStatus getOpModeStatus() {
        return status;
    }

    public void internalInit() {
        status = RobotStatus.OpModeStatus.INIT;
        init();
    }

    public void internalStart() {
        status = RobotStatus.OpModeStatus.RUNNING;
        start();
    }

    public void internalStop() {
        status = RobotStatus.OpModeStatus.STOPPED;
        stop();
    }

    protected abstract void init();

    protected abstract void loop() throws InterruptedException;

    void init_loop() {
    }

    void start() {
    }

    void stop() {
    }

}
