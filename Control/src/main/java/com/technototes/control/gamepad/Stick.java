package com.technototes.control.gamepad;

import com.technototes.control.Periodic;

public interface Stick extends Periodic {
    double getXAxis();
    double getYAxis();
    default double getAngle(){
        return Math.atan2(getYAxis(), getXAxis());
    }
    default double getDistanceFromCenter(){
        return Math.sqrt(getXAxis()*getXAxis()+getYAxis()*getYAxis());
    }
}
