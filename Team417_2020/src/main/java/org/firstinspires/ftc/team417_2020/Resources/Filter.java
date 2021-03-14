package org.firstinspires.ftc.team417_2020.Resources;

/**
    This is implemented by any control filter classes.
*/

public interface Filter
{
    void roll(double newValue);

    double getFilteredValue();
}
