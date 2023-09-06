package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;



public class TweakableDouble extends Tweakable {

    private final double _change_amount;
    public double value;


    public TweakableDouble(String name, double _change_amount, double value) {
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
        value += _change_amount;
    }

    @Override
    public void adjustUp() {
        value -=_change_amount;
    }
}
