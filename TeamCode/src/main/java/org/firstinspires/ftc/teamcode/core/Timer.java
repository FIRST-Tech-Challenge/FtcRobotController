package org.firstinspires.ftc.teamcode.core;

public class Timer {

    // Timer
    public double timeSinceStart;
    public double deltaTime;
    private long currentTime = 0;
    private long tempTime = 0;


    public void Update()
    {
        tempTime = System.currentTimeMillis();
        if (currentTime != 0)
        {
            deltaTime = (tempTime - currentTime) / 1000.0;
        }
        currentTime = tempTime;

        timeSinceStart += deltaTime;
    }

    public void Reset()
    {
        timeSinceStart = 0;
    }

}