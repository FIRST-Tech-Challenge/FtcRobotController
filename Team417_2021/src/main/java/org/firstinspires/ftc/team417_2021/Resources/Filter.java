package org.firstinspires.ftc.team417_2021.Resources;

/**
    This is implemented by any control filter classes.
*/

public interface Filter
{
    void roll(double newValue);

    double getFilteredValue();
}
