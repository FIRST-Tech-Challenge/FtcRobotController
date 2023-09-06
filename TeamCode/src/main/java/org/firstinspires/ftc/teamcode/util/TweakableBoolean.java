package org.firstinspires.ftc.teamcode.util;

public class TweakableBoolean extends Tweakable {
    public boolean value;

    public TweakableBoolean(String name, boolean value) {
        super(name);
        this.value = value;
    }

    @Override
    public String toString() {
        return ""+value;
    }

    @Override
    public void adjustDown() {
        value= !value;
    }

    @Override
    public void adjustUp() {
        value=!value;
    }
}
