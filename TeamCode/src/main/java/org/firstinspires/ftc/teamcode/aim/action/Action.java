package org.firstinspires.ftc.teamcode.aim.action;

public class Action {
    private String name;
    private boolean started = false;
    public Action(String name) {
        this.name = name;

    }
    public boolean run() {
        return true;
    }

    public String toString() {
        return name;
    }

    public String getName() {
        return name;
    }

    protected void markStarted() {
        started = true;
    }

    protected boolean isStarted() {
        return started;
    }
}
