package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

public class TweakableLong extends Tweakable{

    private final long _change_amount;
    public long value;

    public TweakableLong(String name, long _change_amount, long value) {
        super(name);
        this._change_amount = _change_amount;
        this.value = value;
    }

    @NonNull
    @Override
    public String toString() {
        return ""+value;
    }

    @Override
    public void adjustDown() {
        value -= _change_amount;

    }

    @Override
    public void adjustUp() {

        value += _change_amount;
    }
}
