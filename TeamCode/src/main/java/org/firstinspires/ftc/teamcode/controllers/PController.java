package org.firstinspires.ftc.teamcode.controllers;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.utils.MathFunctions;

import java.util.function.Supplier;

public class PController {

    private double kP;

    private double max, min;

    /** Constructor for class P-Controller
     * @param kP    The kP tuning value. Passes as a supplier in order to allow live tuning
     */
    public PController(double kP, double min, double max) {
        this.kP = kP;
        this.max = max;
        this.min = min;
    }

    public double getOutput(double sp, double pv) {
        return MathFunctions.clamp((sp - pv) * kP, max, min);
    }

}
