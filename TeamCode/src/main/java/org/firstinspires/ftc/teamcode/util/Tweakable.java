package org.firstinspires.ftc.teamcode.util;

abstract public class Tweakable {

    abstract public void adjustDown();
    abstract public void adjustUp();
    public String name;

    public Tweakable(String name) {
        this.name = name;
    }
}