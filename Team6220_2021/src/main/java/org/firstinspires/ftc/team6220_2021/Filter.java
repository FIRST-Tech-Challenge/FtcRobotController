package org.firstinspires.ftc.team6220_2021;

/**
 This is implemented by any control filter classes.
 */

public interface Filter
{
    void roll(double newValue);

    double getFilteredValue();
}
